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

using std::lock_guard;
using std::recursive_mutex;
using std::max;
using std::min;
using std::shared_ptr;
using std::vector;

using sigrok::Logic;

namespace pv {
namespace data {

LogicSegment::LogicSegment(pv::data::Logic& owner, uint32_t segment_id,
	unsigned int unit_size,	uint64_t samplerate) :
	Segment(segment_id, samplerate, unit_size),
	owner_(owner)
{
	// Create all sub-signals
	sub_signals_.resize(unit_size_ * 8);

	prev_sample_value_.resize((unit_size + 7) / 8);
}

LogicSegment::~LogicSegment()
{
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

	lock_guard<recursive_mutex> lock(mutex_);

	// Clear memory because we're only going to set bits, not clear any
	memset(dest, 0, (end_sample - start_sample) * unit_size_);

	uint bytepos = 0;
	uint bitpos = 0;

	for (uint si = 0; si < sub_signals_.size(); si++) {
		uint64_t sample_index = 0;
		uint64_t samples_to_create = end_sample - start_sample;

		const RLEData& signal_data = sub_signals_.at(si);
		for (const Edge& change : signal_data.edges) {
			if (sample_index + change.sample_num < (uint64_t)start_sample) {
				// Skip changes until we reach the start sample
				sample_index += change.sample_num;
				continue;
			}

			// Add samples belonging to current edge
			uint8_t edge_value = change.new_state ? (1 << bitpos) : 0;
			uint64_t sample_count = (sample_index < (uint64_t)start_sample) ?
				(start_sample + change.sample_num - sample_index) :
				min(change.sample_num, samples_to_create);

			if (edge_value)
				for (uint i = 0; i < sample_count; i++)
					dest[(sample_index + i) * unit_size_ + bytepos] |= edge_value;

			samples_to_create -= sample_count;
			sample_index += sample_count;
		}

		bitpos++;
		if (bitpos > 7) {
			bytepos++;
			bitpos = 0;
		}
	}
}


void LogicSegment::get_subsampled_edges(
	vector<Edge> &edges,
	uint64_t start, uint64_t end, uint32_t samples_per_pixel,
	uint32_t sig_index, bool first_change_only) const
{
	assert(start <= end);
	assert(sig_index <= unit_size_ * 8);

	if (samples_per_pixel == 0)
		samples_per_pixel = 1;

	lock_guard<recursive_mutex> lock(mutex_);

	// Make sure we only process as many samples as we have
	if (end > get_sample_count())
		end = get_sample_count();

	uint64_t sample_index = 0;
	uint64_t pixel_sample_count = 0;
	bool multi_subsample_edges = false;
	bool last_subsample_edge_state;
	uint64_t multi_subsample_edge_length = 0;

	const RLEData& signal_data = sub_signals_.at(sig_index);

	for (const Edge& change : signal_data.edges) {
		if (sample_index + change.sample_num < start) {
			// Skip changes until we reach the start sample
			sample_index += change.sample_num;
			pixel_sample_count = sample_index % samples_per_pixel;
			continue;
		}

		if (pixel_sample_count + change.sample_num < samples_per_pixel) {
			// Edge occupies the same pixel as the last edge, so don't add it
			multi_subsample_edges = true;
			last_subsample_edge_state = change.new_state;
			multi_subsample_edge_length += change.sample_num;
			pixel_sample_count += change.sample_num;

		} else {
			// Edge is on a new pixel, add previous edge if needed
			if (multi_subsample_edges) {
				edges.emplace_back(sample_index, last_subsample_edge_state);
				sample_index += multi_subsample_edge_length;
				multi_subsample_edges = false;
				multi_subsample_edge_length = 0;
			}

			// Add current edge
			edges.emplace_back(max(sample_index, start), change.new_state);
			sample_index += change.sample_num;
			pixel_sample_count = sample_index % samples_per_pixel;
		}

		if (sample_index >= end)
			break;
	}

	// Make sure at least one edge is present when zoomed out
	if (edges.empty() && samples_per_pixel >= sample_count_)
		edges.emplace_back(0, sub_signals_.at(sig_index).edges.front().new_state);

qDebug() << "-------------------------";
for (Edge& change : edges) qDebug() << change.sample_num << change.new_state;
qDebug() << "-------------------------" << "Index:" << sig_index << "SPP:" << samples_per_pixel << "Edges:" << edges.size();

	(void)first_change_only;
}

void LogicSegment::get_surrounding_edges(vector<Edge> &dest,
	uint64_t origin_sample, uint32_t sig_index) const
{
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
}

void LogicSegment::process_new_samples(void *data, uint64_t samples)
{
	// To iterate over the sample data, we use uint64 to compare up to 64 signals at a time

	// Note: Since we access the sample data using uint64_t* regardless of the
	// unit size, we may access memory beyond the valid range. This must be taken
	// into account by the memory allocator for the sample data.

	uint64_t* run_lengths[unit_size_ * 8];
	bool states[unit_size_ * 8];

	for (uint idx_64 = 0; idx_64 < ((unit_size_ + 7) / 8); idx_64++) {

		const uint start_signal = 64 * idx_64;
		const uint end_signal = std::min(start_signal + 64, sub_signals_.size());

		uint64_t prev_value;

		uint64_t sample_mask = 0;
		for (uint i = start_signal; i < end_signal; i++)
			sample_mask |= ((uint64_t)1 << (i - start_signal));

		uint8_t* ptr = (uint8_t*)data + (8 * idx_64);

		if (sub_signals_.at(start_signal).edges.empty()) {
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
				sub_signals_[si].edges.emplace_back(0, state);
				compare_value <<= 1;
			}
		} else {
			// Restore last used (sub-)sample value and the run lengths
			prev_value = prev_sample_value_[idx_64];
		}

		for (uint si = start_signal; si < end_signal; si++) {
			Edge& e = sub_signals_[si].edges.back();
			run_lengths[si] = &(sub_signals_[si].edges.back().sample_num);
			states[si] = e.new_state;
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
						sub_signals_[si].edges.emplace_back(1, state);
						run_lengths[si] = &(sub_signals_[si].edges.back().sample_num);
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
