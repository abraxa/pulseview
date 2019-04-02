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

	append_samples(data, sample_count);
	process_new_samples(data, sample_count);

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

	get_raw_samples(start_sample, (end_sample - start_sample), dest);
}

void LogicSegment::get_subsampled_edges(
	vector<EdgePair> &edges,
	uint64_t start, uint64_t end, uint32_t samples_per_pixel,
	uint32_t sig_index, bool first_change_only)
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

	for (EdgePair& change : sub_signals_.at(sig_index)) {
		if (sample_index + change.first < start) {
			// Skip changes until we reach the start sample
			sample_index += change.first;
			pixel_sample_count = sample_index % samples_per_pixel;
			continue;
		}

		if (pixel_sample_count + change.first < samples_per_pixel) {
			// Edge occupies the same pixel as the last edge, so don't add it
			multi_subsample_edges = true;
			last_subsample_edge_state = change.second;
			multi_subsample_edge_length += change.first;
			pixel_sample_count += change.first;

		} else {
			// Edge is on a new pixel, add previous edge if needed
			if (multi_subsample_edges) {
				edges.emplace_back(sample_index, last_subsample_edge_state);
				sample_index += multi_subsample_edge_length;
				multi_subsample_edges = false;
				multi_subsample_edge_length = 0;
			}

			// Add current edge
			edges.emplace_back(max(sample_index, start), change.second);
			sample_index += change.first;
			pixel_sample_count = sample_index % samples_per_pixel;
		}

		if (sample_index >= end)
			break;
	}

	// Make sure at least one edge is present when zoomed out
	if (edges.empty() && samples_per_pixel >= sample_count_)
		edges.emplace_back(0, sub_signals_.at(sig_index).front().second);

qDebug() << "-------------------------";
for (EdgePair& change : edges) qDebug() << change.first << change.second;
qDebug() << "-------------------------" << "Index:" << sig_index << "SPP:" << samples_per_pixel << "Edges:" << edges.size();

	(void)first_change_only;
}

void LogicSegment::get_surrounding_edges(vector<EdgePair> &dest,
	uint64_t origin_sample, uint32_t sig_index)
{
	if (origin_sample >= sample_count_)
		return;

	// Put the edges vector on the heap, it can become quite big until we can
	// use a get_subsampled_edges() implementation that searches backwards
	vector<EdgePair>* edges = new vector<EdgePair>;

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
	uint64_t sample_mask = 0;
	for (uint8_t i = 0; i < (unit_size_ * 8); i++)
		sample_mask += (1 << i);

	for (uint64_t i = 0; i < samples; i++) {
		const uint64_t* ptr = (uint64_t*)( (uint64_t)data + i * unit_size_ );
		const uint64_t sample_value = (*ptr) & sample_mask;

		if (sub_signals_.front().empty()) {
			// Sub-signals are empty, use the current state as initial state
			prev_sample_value_ = sample_value;

			uint64_t compare_value = 1;
			for (uint32_t si = 0; si < sub_signals_.size(); si++) {
				// Add the first RLE entry to all sub-signals and set the states
				// of all sub-signal RLE entries to the current state
				bool state = (sample_value & compare_value) == compare_value;
				sub_signals_.at(si).emplace_back(1, state);
				compare_value <<= 1;
			}
			continue;
		}

		if (sample_value == prev_sample_value_) {
			// Sample value hasn't changed, simply increase all run lengths
			for (uint32_t si = 0; si < sub_signals_.size(); si++)
				sub_signals_.at(si).back().first++;
		} else {
			// Demux sample and increase sub-signal run length or insert state change
			uint64_t compare_value = 1;
			for (uint32_t si = 0; si < sub_signals_.size(); si++) {
				bool state = (sample_value & compare_value) == compare_value;

				if (state == sub_signals_.at(si).back().second)
					sub_signals_.at(si).back().first++;
				else
					sub_signals_.at(si).emplace_back(1, state);

				compare_value <<= 1;
			}
			prev_sample_value_ = sample_value;
		}
	}
}

} // namespace data
} // namespace pv
