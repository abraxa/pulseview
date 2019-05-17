/*
 * This file is part of the PulseView project.
 *
 * Copyright (C) 2012 Joel Holdsworth <joel@airwebreathe.org.uk>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#include "config.h" // For HAVE_UNALIGNED_LITTLE_ENDIAN_ACCESS

#include <extdef.h>

#include <cassert>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <cstdint>

#include "logic.hpp"
#include "logicsegment.hpp"

#include <libsigrokcxx/libsigrokcxx.hpp>

#include <QDebug>
#include <QThread>
#include <QtConcurrent/QtConcurrentRun>

using std::lock_guard;
using std::recursive_mutex;
using std::max;
using std::min;
using std::shared_ptr;
using std::vector;

using sigrok::Logic;

namespace pv {
namespace data {

#define LOGICSEGMENT_DEBUG (0)

const uint LogicSegment::chunkSize = 10240;  /* Maximum number of edges stored in a chunk */
                                             /* TBD: Make this dependent on the number of channels */

LogicSegment::LogicSegment(pv::data::Logic& owner, uint32_t segment_id,
	unsigned int unit_size,	uint64_t samplerate) :
	Segment(segment_id, samplerate, unit_size),
	owner_(owner)
{
	// Prepare to receive samples and hold the previous states
	prev_sample_value_.resize((unit_size + 7) / 8);

	// Create all sub-signals
	sub_signals_.resize(unit_size_ * 8);

	for (uint i = 0; i < (unit_size * 8); i++) {
		RLEData* signal = &sub_signals_[i];
		signal->current_chunk = new Edge[chunkSize];
		signal->edge_chunks.push_back(signal->current_chunk);
		signal->chunk_start_samples.push_back(0);
		signal->used_edges = 0;
		signal->unused_edges = chunkSize;
	}
}

LogicSegment::~LogicSegment()
{
	for (RLEData& signal : sub_signals_)
		for (Edge* chunk : signal.edge_chunks)
			delete[] chunk;
}

void LogicSegment::append_payload(shared_ptr<sigrok::Logic> logic)
{
	assert(unit_size_ == logic->unit_size());
	assert((logic->data_length() % unit_size_) == 0);

	append_payload(logic->data_pointer(), logic->data_length());
}

void LogicSegment::append_payload(void *data, uint64_t data_size)
{
	assert((data_size % unit_size_) == 0);

	lock_guard<recursive_mutex> lock(mutex_);

	const uint64_t prev_sample_count = sample_count_;
	const uint64_t sample_count = data_size / unit_size_;

	process_new_samples(data, sample_count);
	sample_count_ += sample_count;

	if (sample_count > 1)
		owner_.notify_samples_added(this, prev_sample_count + 1,
			prev_sample_count + 1 + sample_count);
	else
		owner_.notify_samples_added(this, prev_sample_count + 1,
			prev_sample_count + 1);
}

void LogicSegment::get_samples(int64_t start_sample,
	int64_t end_sample, uint8_t* dest) const
{
	assert(start_sample >= 0);
	assert(start_sample <= (int64_t)sample_count_);
	assert(end_sample >= 0);
	assert(end_sample <= (int64_t)sample_count_);
	assert(start_sample <= end_sample);
	assert(dest != nullptr);

	// Clear memory because we're only going to set bits, not clear any
	memset(dest, 0, (end_sample - start_sample) * unit_size_);

	uint bytepos = 0;
	uint bitpos = 0;

	for (uint si = 0; si < sub_signals_.size(); si++) {

		add_subsignal_to_data(start_sample, end_sample, dest, unit_size_,
			si, bitpos, bytepos);

		bitpos++;
		if (bitpos > 7) {
			bytepos++;
			bitpos = 0;
		}
	}
}

void LogicSegment::add_subsignal_to_data(int64_t start_sample, int64_t end_sample,
	uint8_t* dest, uint8_t dest_unit_size, uint signal_idx, uint8_t bitpos, uint8_t bytepos) const
{
	assert(start_sample >= 0);
	assert(start_sample <= (int64_t)sample_count_);
	assert(end_sample >= 0);
	assert(end_sample <= (int64_t)sample_count_);
	assert(start_sample <= end_sample);
	assert(dest != nullptr);
	assert(dest_unit_size > 0);
	assert(signal_idx < sub_signals_.size());

	lock_guard<recursive_mutex> lock(mutex_);

	uint64_t sample_index = 0;
	uint64_t samples_to_create = end_sample - start_sample;
	uint64_t dest_idx = bytepos;

	const RLEData& signal_data = sub_signals_.at(signal_idx);

	for (const Edge* chunk: signal_data.edge_chunks) {
		uint chunk_size = (chunk == signal_data.current_chunk) ?
			signal_data.used_edges : chunkSize;

		for (uint edge_idx = 0; edge_idx < chunk_size;  edge_idx++) {
			const Edge* change = &(chunk[edge_idx]);

			if (sample_index + change->sample_num <= (uint64_t)start_sample) {
				// Skip changes until we reach the start sample
				sample_index += change->sample_num;
				continue;
			}

			// Add samples belonging to current edge
			uint8_t edge_value = change->new_state ? (1 << bitpos) : 0;
			uint64_t sample_count = (sample_index < (uint64_t)start_sample) ?
				(change->sample_num - (start_sample - sample_index)) :
				change->sample_num;
			sample_count = min(sample_count, samples_to_create);

			if (edge_value) {
				for (uint i = 0; i < sample_count; i++) {
					assert(dest_idx < (end_sample - start_sample) * dest_unit_size);
					dest[dest_idx] |= edge_value;
					dest_idx += dest_unit_size;
				}
			} else
				dest_idx += sample_count * dest_unit_size;

			samples_to_create -= sample_count;
			sample_index += change->sample_num;

			if (samples_to_create == 0)
				break;
		}
	}
}

void LogicSegment::get_subsampled_edges_worker(vector<Edge> *edges,
	uint64_t start, uint64_t end, double samples_per_pixel, uint32_t sig_index)
{
	assert(start <= end);
	assert(sig_index <= unit_size_ * 8);

	const RLEData& rle_data = sub_signals_.at(sig_index);

	uint start_chunk_id = 0;
	uint end_chunk_id = 0;

	// Determine start and end chunks
	for (uint chunk_id = 0; chunk_id < rle_data.chunk_start_samples.size(); chunk_id++) {
		uint64_t chunk_start_sample = rle_data.chunk_start_samples[chunk_id];

		if ((start != 0) && (chunk_start_sample <= start))
			start_chunk_id = chunk_id;
		if (chunk_start_sample <= end)
			end_chunk_id = chunk_id;
		else
			break;
	}
#if LOGICSEGMENT_DEBUG
	qDebug() << "  Worker: start/end:" << start << end << "; chunk ID is" << start_chunk_id \
			 << "with max" << (rle_data.edge_chunks.size() - 1) << "edges";
#endif

	// We only process a single chunk, so make sure start/end chunk are the same
	assert(start_chunk_id == end_chunk_id);
	assert(start_chunk_id < rle_data.edge_chunks.size());

	if (samples_per_pixel == 0)
		samples_per_pixel = 1;

	// For now, we perform 3x oversampling to reduce artifacts at the cost of speed
	samples_per_pixel /= 3;

	// Make sure we only process as many samples as we have
	if (end > get_sample_count())
		end = get_sample_count();

	const Edge* cur_edge = rle_data.edge_chunks[start_chunk_id];
	uint64_t sample_index = rle_data.chunk_start_samples[start_chunk_id];

	// We don't start the scaling counter at 0 because that might produce gaps
	// where two chunks meet, depending on the scaling factor. Instead, we
	// use the start sample to determine where to resume the scaling
	uint64_t pixel_sample_count = 0;
	if (sample_index > 0)
		pixel_sample_count = (sample_index - 1) % max((int)samples_per_pixel, 1);

	bool multi_subsample_edges = false;
	bool last_subsample_edge_state;
	uint64_t multi_subsample_edge_length = 0;
	Edge e;

	for (uint i = 0; i < chunkSize; i++, cur_edge++) {
		if (sample_index + cur_edge->sample_num < start) {
			// Skip changes until we reach the start sample
			sample_index += cur_edge->sample_num;
			pixel_sample_count = sample_index % max((int)samples_per_pixel, 1);
			continue;
		}

		if (pixel_sample_count + cur_edge->sample_num < samples_per_pixel) {
			// Edge occupies the same pixel as the last edge, so don't add it
			multi_subsample_edges = true;
			last_subsample_edge_state = cur_edge->new_state;
			multi_subsample_edge_length += cur_edge->sample_num;
			pixel_sample_count += cur_edge->sample_num;

		} else {
			// Edge is on a new pixel, add previous edge if needed
			if (multi_subsample_edges) {
				e.sample_num = sample_index;
				e.new_state = last_subsample_edge_state;
				edges->push_back(e);
				sample_index += multi_subsample_edge_length;
				multi_subsample_edges = false;
				multi_subsample_edge_length = 0;
			}

			// Add current edge
			e.sample_num = max(sample_index, start);
			e.new_state = cur_edge->new_state;
			edges->push_back(e);
			sample_index += cur_edge->sample_num;
			pixel_sample_count = sample_index % max((int)samples_per_pixel, 1);  // TODO Can this be replaced by a subtraction?
		}

		if ((sample_index > end) || (cur_edge->sample_num == 0)) {
			// Note: RLE length is only 0 when the RLE entry is empty. Can happen
			// when the last RLE entry doesn't reach the value of end because of scaling.
			break;
		}
	}

	// Make sure at least one edge is present when zoomed out
	if (edges->empty() && samples_per_pixel >= sample_count_) {
		e.sample_num = 0;
		e.new_state = sub_signals_.at(sig_index).edge_chunks[start_chunk_id][0].new_state;
		edges->push_back(e);
	}

#if LOGICSEGMENT_DEBUG
	qDebug() << "  Processing" << start << "to" << end << "yielded" << edges->size() << "edges for chunk" << start_chunk_id;
#endif
}

void LogicSegment::get_subsampled_edges(
	vector<Edge> &edges,
	uint64_t start, uint64_t end, double samples_per_pixel,
	uint32_t sig_index, bool first_change_only)
{
	const RLEData& rle_data = sub_signals_.at(sig_index);

	if (first_change_only) {
		// TBD: first_change_only implementation missing
		//get_subsampled_edges_worker(&edges, start, end, samples_per_pixel, sig_index);
		return;
	}

	vector< QFuture<void> > threads;
	vector< vector<Edge>* > thread_edges;
	vector<uint64_t> thread_end_samples;
	uint64_t s = start;
	uint64_t e;

	// Break up the work load by chunks, processed in separate threads.
	// Chunks are not only used because of optimized memory allocation but
	// also because chunks will never move around in memory like a vector's
	// underlying data would. Hence, no locking is needed when reading from
	// them even when the aquisition is still running.

	uint start_chunk_id = 0;
	uint end_chunk_id = 0;

	// Determine start and end chunks
	for (uint chunk_id = 0; chunk_id < rle_data.chunk_start_samples.size(); chunk_id++) {
		uint64_t chunk_start_sample = rle_data.chunk_start_samples[chunk_id];

		if ((start != 0) && (chunk_start_sample <= start))
			start_chunk_id = chunk_id;
		if (chunk_start_sample <= end)
			end_chunk_id = chunk_id;
		else
			break;
	}

#if LOGICSEGMENT_DEBUG
	qDebug() << "--------------------------------------------------";
	qDebug() << "Whole range / chunks:" << start << end << "/" << start_chunk_id << end_chunk_id;
#endif

	if (start_chunk_id != end_chunk_id) {
		// Process first incomplete chunk
		uint64_t next_chunk_start = rle_data.chunk_start_samples[start_chunk_id + 1];
		e = next_chunk_start - 1;  // End of chunk the start sample is in
#if LOGICSEGMENT_DEBUG
		qDebug() << "First incomplete chunk:" << s << e;
#endif
		thread_edges.push_back(new vector<Edge>());
		QFuture<void> f = QtConcurrent::run(this, &LogicSegment::get_subsampled_edges_worker,
			thread_edges.back(), s, e, samples_per_pixel, sig_index);
		thread_end_samples.push_back(e);
		threads.push_back(f);
		start_chunk_id++;
		s = next_chunk_start;  // Beginning of next chunk

#if LOGICSEGMENT_DEBUG
		for (QFuture<void>& f : threads) f.waitForFinished();
#endif

		// Process complete chunks inbetween if there are any
		while (start_chunk_id < end_chunk_id) {
			uint64_t next_chunk_start = rle_data.chunk_start_samples[start_chunk_id + 1];
			e = next_chunk_start - 1;  // End of chunk the start sample is in
#if LOGICSEGMENT_DEBUG
			qDebug() << "Complete chunk:" << s << e << "/" << start_chunk_id;
#endif
			thread_edges.push_back(new vector<Edge>());
			QFuture<void> f = QtConcurrent::run(this, &LogicSegment::get_subsampled_edges_worker,
				thread_edges.back(), s, e, samples_per_pixel, sig_index);
			thread_end_samples.push_back(e);
			threads.push_back(f);

			s = next_chunk_start;
			start_chunk_id++;

#if LOGICSEGMENT_DEBUG
			for (QFuture<void>& f : threads) f.waitForFinished();
#endif
		}

		// Process last incomplete chunk directly (start_chunk_id is now end_chunk_id)
		e = end;
		thread_edges.push_back(new vector<Edge>());
#if LOGICSEGMENT_DEBUG
		qDebug() << "Last incomplete chunk:" << s << e;
#endif
		get_subsampled_edges_worker(thread_edges.back(), s, e, samples_per_pixel, sig_index);
		thread_end_samples.push_back(e);

	} else {
		// Single chunk, no need to start a thread
		thread_edges.push_back(new vector<Edge>());
		get_subsampled_edges_worker(thread_edges.back(), start, end, samples_per_pixel, sig_index);
		thread_end_samples.push_back(end);
	}

	// Wait for all threads to finish
	for (QFuture<void>& f : threads)
		f.waitForFinished();

	// Join all edges extracted from the processed blocks
	unsigned int thread_index = 0;
	for (vector<Edge>* te : thread_edges) {
		edges.reserve(edges.size() + te->size());

		for (auto e_it = te->begin(); e_it != te->end(); e_it++) {
			edges.emplace_back(*e_it);
		}

		delete te;
		thread_index++;
	}
}

void LogicSegment::get_surrounding_edges(vector<Edge> &dest,
	uint64_t origin_sample, uint32_t sig_index)
{
	(void)dest; (void)origin_sample; (void)sig_index;

	/*
	if (origin_sample >= sample_count_)
		return;

	// Put the edges vector on the heap, it can become quite big until we can
	// use a get_subsampled_edges() implementation that searches backwards
	vector<Edge>* edges = new vector<Edge>;

	// Get all edges to the left of origin_sample
	get_subsampled_edges(*edges, 0, origin_sample, sig_index, false);

	// If we don't specify "first only", the first and last edge are the states
	// at samples 0 and origin_sample. If only those exist, there are no edges
	if (edges->size() == 2) {
		delete edges;
		return;
	}

	// Dismiss the entry for origin_sample so that back() gives us the
	// real last entry
	edges->pop_back();
	dest.push_back(edges->back());
	edges->clear();

	// Get first edge to the right of origin_sample
	get_subsampled_edges(*edges, origin_sample, sample_count_, sig_index, true);

	// "first only" is specified, so nothing needs to be dismissed
	if (edges->size() == 0) {
		delete edges;
		return;
	}

	dest.push_back(edges->front());

	delete edges;
	*/
}

LogicSegment::Edge* LogicSegment::add_state_to_sub_signal(RLEData* rle_data,
	uint64_t samplenum, bool state, uint64_t length)
{
	uint64_t prev_edge_sample = 0;

	if (rle_data->unused_edges == 0) {
		rle_data->chunk_last_edge_samples.push_back(prev_edge_sample);

		rle_data->current_chunk = new Edge[chunkSize];
		rle_data->edge_chunks.push_back(rle_data->current_chunk);
		rle_data->chunk_start_samples.push_back(samplenum);
		rle_data->used_edges = 0;
		rle_data->unused_edges = chunkSize;
#if LOGICSEGMENT_DEBUG
		if (rle_data == &sub_signals_.at(0)) qDebug() << "Added new chunk at samplenum" \
			<< samplenum << ", now have" << (rle_data->chunk_start_samples.size()) << "chunks";
		if (rle_data == &sub_signals_.at(0)) qDebug() << "New chunk start sample is" \
			<< (rle_data->chunk_start_samples.back());
#endif
	}

	Edge* e = &(rle_data->current_chunk[rle_data->used_edges]);

#if LOGICSEGMENT_DEBUG
	if (rle_data == &sub_signals_.at(5)) qDebug() << " new RLE value:" << samplenum << "->" << state;
#endif

	e->sample_num = length;
	e->new_state = state;

	rle_data->used_edges++;
	rle_data->edge_count++;
	rle_data->unused_edges--;

	prev_edge_sample = samplenum;

	return e;
}

void LogicSegment::process_new_samples(void *data, uint64_t samples)
{
	// To iterate over the sample data, we use uint64 to compare up to 64 signals at a time

	// Note: Since we access the sample data using uint64_t* regardless of the
	// unit size, we may access memory beyond the valid range. This must be taken
	// into account by the memory allocator for the sample data.

	uint32_t* run_lengths[unit_size_ * 8];
	bool states[unit_size_ * 8];

	for (uint idx_64 = 0; idx_64 < ((unit_size_ + 7) / 8); idx_64++) {

		const uint start_signal = 64 * idx_64;
		const uint end_signal = std::min(start_signal + 64, (uint)sub_signals_.size());

		uint64_t prev_value;

		uint64_t sample_mask = 0;
		for (uint i = start_signal; i < end_signal; i++)
			sample_mask |= ((uint64_t)1 << (i - start_signal));

		uint8_t* ptr = (uint8_t*)data + (8 * idx_64);

		if (sub_signals_.at(start_signal).edge_count == 0) {
			// Sub-signals are empty, use the current state as initial state
			const uint64_t sample_value = (*(uint64_t*)ptr) & sample_mask;

			prev_sample_value_.push_back(sample_value);
			prev_value = sample_value;

			uint64_t compare_value = 1;
			for (uint si = start_signal; si < end_signal; si++) {
				// Add the first RLE entry to all sub-signals and set the states
				// of all sub-signal RLE entries to the current state
				bool state = (sample_value & compare_value) == compare_value;
				// Note: 0 because this same sample will be processed again, don't count it twice
				RLEData* rle_data = &sub_signals_[si];
				add_state_to_sub_signal(rle_data, 0, state, 0);
				compare_value <<= 1;
			}
		} else {
			// Restore last used (sub-)sample value and the run lengths
			prev_value = prev_sample_value_[idx_64];
		}

		// Load the last states present in the sub-signals
		for (uint si = start_signal; si < end_signal; si++) {
			RLEData* rle_data = &sub_signals_[si];
			Edge& last_edge = rle_data->current_chunk[rle_data->used_edges - 1];
			run_lengths[si] = &last_edge.sample_num;
			states[si] = last_edge.new_state;
		}

		// This variable is used to quickly handle samples that don't change
		// in value. Instead of iterating over all signals for all samples
		// and increasing each run length by 1, we accumulate the length of
		// all non-changing samples and add them in one go
		uint64_t same_sample_count = 0;

		for (uint64_t i = 0; i < samples; i++) {
			const uint64_t sample_value = (*(uint64_t*)ptr) & sample_mask;

			if (sample_value == prev_value) {
				// Sample value hasn't changed
				same_sample_count++;
			} else {
				// Sample changed, update run lengths of previous sample using
				// the accumulated same-sample count
				if (same_sample_count > 0) {
					for (uint si = start_signal; si < end_signal; si++)
						(*run_lengths[si]) += same_sample_count;
					same_sample_count = 0;
				}

				// Demux sample and increase sub-signal run length or insert state change
				uint64_t compare_value = 1;
				for (uint si = start_signal; si < end_signal; si++) {
					bool state = (sample_value & compare_value) == compare_value;

					if (state == states[si])
						(*run_lengths[si])++;
					else {
						RLEData* rle_data = &sub_signals_[si];
						Edge* e = add_state_to_sub_signal(rle_data, sample_count_ + i, state, 1);
						run_lengths[si] = &(e->sample_num);
						states[si] = state;
					}

					compare_value <<= 1;
				}
				prev_value = sample_value;
			}

			ptr += unit_size_;
		}

		// If necessary, update run lengths with accumulated same-sample count
		if (same_sample_count > 0)
			for (uint si = start_signal; si < end_signal; si++)
				(*run_lengths[si]) += same_sample_count;

		// Save last used (sub-)sample value
		prev_sample_value_[idx_64] = prev_value;
	}
}

} // namespace data
} // namespace pv
