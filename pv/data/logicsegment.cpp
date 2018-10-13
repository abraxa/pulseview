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
	owner_(owner),
	last_append_sample_(0),
	last_append_accumulator_(0),
	last_append_extra_(0)
{
	// Create all sub-signals
	sub_signals_.resize(unit_size_ * 8);
}

LogicSegment::~LogicSegment()
{
}

template <class T>
void LogicSegment::downsampleTmain(const T*&in, T &acc, T &prev)
{
	// Accumulate one sample at a time
	for (uint64_t i = 0; i < MipMapScaleFactor; i++) {
		T sample = *in++;
		acc |= prev ^ sample;
		prev = sample;
	}
}

template <>
void LogicSegment::downsampleTmain<uint8_t>(const uint8_t*&in, uint8_t &acc, uint8_t &prev)
{
	// Handle 8 bit samples in 32 bit steps
	uint32_t prev32 = prev | prev << 8 | prev << 16 | prev << 24;
	uint32_t acc32 = acc;
	const uint32_t *in32 = (const uint32_t*)in;
	for (uint64_t i = 0; i < MipMapScaleFactor; i += 4) {
		uint32_t sample32 = *in32++;
		acc32 |= prev32 ^ sample32;
		prev32 = sample32;
	}
	// Reduce result back to uint8_t
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
	prev = (prev32 >> 24) & 0xff; // MSB is last
#elif __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
	prev = prev32 & 0xff; // LSB is last
#else
#error Endianness unknown
#endif
	acc |= acc32 & 0xff;
	acc |= (acc32 >> 8) & 0xff;
	acc |= (acc32 >> 16) & 0xff;
	acc |= (acc32 >> 24) & 0xff;
	in = (const uint8_t*)in32;
}

template <>
void LogicSegment::downsampleTmain<uint16_t>(const uint16_t*&in, uint16_t &acc, uint16_t &prev)
{
	// Handle 16 bit samples in 32 bit steps
	uint32_t prev32 = prev | prev << 16;
	uint32_t acc32 = acc;
	const uint32_t *in32 = (const uint32_t*)in;
	for (uint64_t i = 0; i < MipMapScaleFactor; i += 2) {
		uint32_t sample32 = *in32++;
		acc32 |= prev32 ^ sample32;
		prev32 = sample32;
	}
	// Reduce result back to uint16_t
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
	prev = (prev32 >> 16) & 0xffff; // MSB is last
#elif __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
	prev = prev32 & 0xffff; // LSB is last
#else
#error Endian unknown
#endif
	acc |= acc32 & 0xffff;
	acc |= (acc32 >> 16) & 0xffff;
	in = (const uint16_t*)in32;
}

template <class T>
void LogicSegment::downsampleT(const uint8_t *in_, uint8_t *&out_, uint64_t len)
{
	const T *in = (const T*)in_;
	T *out = (T*)out_;
	T prev = last_append_sample_;
	T acc = last_append_accumulator_;

	// Try to complete the previous downsample
	if (last_append_extra_) {
		while (last_append_extra_ < MipMapScaleFactor && len > 0) {
			T sample = *in++;
			acc |= prev ^ sample;
			prev = sample;
			last_append_extra_++;
			len--;
		}
		if (!len) {
			// Not enough samples available to complete downsample
			last_append_sample_ = prev;
			last_append_accumulator_ = acc;
			return;
		}
		// We have a complete downsample
		*out++ = acc;
		acc = 0;
		last_append_extra_ = 0;
	}

	// Handle complete blocks of MipMapScaleFactor samples
	while (len >= MipMapScaleFactor) {
		downsampleTmain<T>(in, acc, prev);
		len -= MipMapScaleFactor;
		// Output downsample
		*out++ = acc;
		acc = 0;
	}

	// Process remainder, not enough for a complete sample
	while (len > 0) {
		T sample = *in++;
		acc |= prev ^ sample;
		prev = sample;
		last_append_extra_++;
		len--;
	}

	// Update context
	last_append_sample_ = prev;
	last_append_accumulator_ = acc;
	out_ = (uint8_t *)out;
}

void LogicSegment::downsampleGeneric(const uint8_t *in, uint8_t *&out, uint64_t len)
{
	// Downsample using the generic unpack_sample()
	// which can handle any width between 1 and 8 bytes
	uint64_t prev = last_append_sample_;
	uint64_t acc = last_append_accumulator_;

	// Try to complete the previous downsample
	if (last_append_extra_) {
		while (last_append_extra_ < MipMapScaleFactor && len > 0) {
			const uint64_t sample = unpack_sample(in);
			in += unit_size_;
			acc |= prev ^ sample;
			prev = sample;
			last_append_extra_++;
			len--;
		}
		if (!len) {
			// Not enough samples available to complete downsample
			last_append_sample_ = prev;
			last_append_accumulator_ = acc;
			return;
		}
		// We have a complete downsample
		pack_sample(out, acc);
		out += unit_size_;
		acc = 0;
		last_append_extra_ = 0;
	}

	// Handle complete blocks of MipMapScaleFactor samples
	while (len >= MipMapScaleFactor) {
		// Accumulate one sample at a time
		for (uint64_t i = 0; i < MipMapScaleFactor; i++) {
			const uint64_t sample = unpack_sample(in);
			in += unit_size_;
			acc |= prev ^ sample;
			prev = sample;
		}
		len -= MipMapScaleFactor;
		// Output downsample
		pack_sample(out, acc);
		out += unit_size_;
		acc = 0;
	}

	// Process remainder, not enough for a complete sample
	while (len > 0) {
		const uint64_t sample = unpack_sample(in);
		in += unit_size_;
		acc |= prev ^ sample;
		prev = sample;
		last_append_extra_++;
		len--;
	}

	// Update context
	last_append_sample_ = prev;
	last_append_accumulator_ = acc;
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
	uint64_t start, uint64_t end,
	uint32_t sig_index, bool first_change_only)
{
	assert(start <= end);

	lock_guard<recursive_mutex> lock(mutex_);

	// Make sure we only process as many samples as we have
	if (end > get_sample_count())
		end = get_sample_count();

	(void)edges;
	(void)sig_index;
	(void)first_change_only;

/*	const uint64_t block_length = (uint64_t)max(min_length, 1.0f);
	const unsigned int min_level = max((int)floorf(logf(min_length) /
		LogMipMapScaleFactor) - 1, 0);
	const uint64_t sig_mask = 1ULL << sig_index;

	// Store the initial state
	last_sample = (get_unpacked_sample(start) & sig_mask) != 0;
	if (!first_change_only)
		edges.emplace_back(index++, last_sample);


	// Add the final state
	if (!first_change_only) {
		const bool end_sample = get_unpacked_sample(end) & sig_mask;
		if (last_sample != end_sample)
			edges.emplace_back(end, end_sample);
		edges.emplace_back(end + 1, end_sample);
	}
	*/
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

			const uint64_t sample = unpack_sample(it->value);
			accumulator |= last_append_sample_ ^ sample;
			continue_raw_sample_iteration(it, 1);
		else if (unit_size_ == 4)
		pack_sample(dest_ptr, accumulator);
		dest_ptr += unit_size_;
			downsampleT<uint8_t>(src_ptr, dest_ptr, count);
		else
			downsampleGeneric(src_ptr, dest_ptr, count);
		len_sample -= count;
		// Advance iterator, should move to start of next chunk
		continue_sample_iteration(it, count);
	end_raw_sample_iteration(it);
uint64_t LogicSegment::get_unpacked_sample(uint64_t index) const
{
	assert(index < sample_count_);

	assert(unit_size_ <= 8);  // 8 * 8 = 64 channels
	uint8_t data[8];

	get_raw_samples(index, 1, data);

//	return unpack_sample(data);
	return 0;
}

void LogicSegment::process_new_samples(void *data, uint64_t samples)
{
	for (uint64_t i = 0; i < samples; i++) {
		if (sub_signals_.front().empty()) {
			prev_sample_value_ = unpack_sample(data);
		}
	}
}
}


} // namespace data
} // namespace pv
