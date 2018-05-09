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

#include <extdef.h>

#include <cassert>
#include <cmath>
#include <cstdlib>
#include <cstring>

#include <QDebug>

#include "logic.hpp"
#include "logicsegment.hpp"

#include <libsigrokcxx/libsigrokcxx.hpp>

using std::lock_guard;
using std::recursive_mutex;
using std::max;
using std::min;
using std::shared_ptr;
using std::vector;

using sigrok::Logic;

namespace pv {
namespace data {

const int LogicSegment::MipMapScalePower = 4;
const int LogicSegment::MipMapScaleFactor = 1 << MipMapScalePower;
const float LogicSegment::LogMipMapScaleFactor = logf(MipMapScaleFactor);
const uint64_t LogicSegment::MipMapDataUnit = 64 * 1024; // bytes

LogicSegment::LogicSegment(pv::data::Logic& owner, uint32_t segment_id,
	unsigned int unit_size,	uint64_t samplerate) :
	Segment(segment_id, samplerate, unit_size),
	owner_(owner),
	last_append_sample_(0)
{
	memset(mip_map_, 0, sizeof(mip_map_));
}

LogicSegment::~LogicSegment()
{
	lock_guard<recursive_mutex> lock(mutex_);
	for (MipMapLevel &l : mip_map_)
		free(l.data);
}

uint64_t LogicSegment::unpack_sample(const uint8_t *ptr) const
{
#ifdef HAVE_UNALIGNED_LITTLE_ENDIAN_ACCESS
	return *(uint64_t*)ptr;
#else
	uint64_t value = 0;
	switch (unit_size_) {
	default:
		value |= ((uint64_t)ptr[7]) << 56;
		/* FALLTHRU */
	case 7:
		value |= ((uint64_t)ptr[6]) << 48;
		/* FALLTHRU */
	case 6:
		value |= ((uint64_t)ptr[5]) << 40;
		/* FALLTHRU */
	case 5:
		value |= ((uint64_t)ptr[4]) << 32;
		/* FALLTHRU */
	case 4:
		value |= ((uint32_t)ptr[3]) << 24;
		/* FALLTHRU */
	case 3:
		value |= ((uint32_t)ptr[2]) << 16;
		/* FALLTHRU */
	case 2:
		value |= ptr[1] << 8;
		/* FALLTHRU */
	case 1:
		value |= ptr[0];
		/* FALLTHRU */
	case 0:
		break;
	}
	return value;
#endif
}

void LogicSegment::pack_sample(uint8_t *ptr, uint64_t value)
{
#ifdef HAVE_UNALIGNED_LITTLE_ENDIAN_ACCESS
	*(uint64_t*)ptr = value;
#else
	switch (unit_size_) {
	default:
		ptr[7] = value >> 56;
		/* FALLTHRU */
	case 7:
		ptr[6] = value >> 48;
		/* FALLTHRU */
	case 6:
		ptr[5] = value >> 40;
		/* FALLTHRU */
	case 5:
		ptr[4] = value >> 32;
		/* FALLTHRU */
	case 4:
		ptr[3] = value >> 24;
		/* FALLTHRU */
	case 3:
		ptr[2] = value >> 16;
		/* FALLTHRU */
	case 2:
		ptr[1] = value >> 8;
		/* FALLTHRU */
	case 1:
		ptr[0] = value;
		/* FALLTHRU */
	case 0:
		break;
	}
#endif
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

	uint64_t prev_sample_count = sample_count_;
	uint64_t sample_count = data_size / unit_size_;

	append_samples(data, sample_count);

	// Generate the first mip-map from the data
	append_payload_to_mipmap(prev_sample_count, prev_sample_count + sample_count,
		(const uint8_t*)data);

	if (sample_count > 1)
		owner_.notify_samples_added(this, prev_sample_count + 1,
			prev_sample_count + 1 + sample_count);
	else
		owner_.notify_samples_added(this, prev_sample_count + 1,
			prev_sample_count + 1);
}

void LogicSegment::get_samples(int64_t start_sample,
	int64_t end_sample,	uint8_t* dest)
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

void LogicSegment::reallocate_mipmap_level(MipMapLevel &m)
{
	lock_guard<recursive_mutex> lock(mutex_);

	const uint64_t new_data_length = ((m.length + MipMapDataUnit - 1) /
		MipMapDataUnit) * MipMapDataUnit;

	if (new_data_length > m.data_length) {
		m.data_length = new_data_length;

		// Padding is added to allow for the uint64_t write word
		m.data = realloc(m.data, new_data_length * unit_size_ +
			sizeof(uint64_t));
	}
}

void LogicSegment::append_payload_to_mipmap(uint64_t p_start_sample, uint64_t p_end_sample,
	const uint8_t *payload)
{
	MipMapLevel &m0 = mip_map_[0];
	uint64_t prev_length;
	uint8_t *dest_ptr;
	uint64_t accumulator;
	unsigned int diff_counter;

	// Expand the data buffer to fit the new samples
	prev_length = m0.length;
	m0.length = sample_count_ / MipMapScaleFactor;

	// Break off if there are no new samples to compute
	if (m0.length == prev_length)
		return;

	reallocate_mipmap_level(m0);

	dest_ptr = (uint8_t*)m0.data + prev_length * unit_size_;

	// Iterate through the samples to populate the first level mipmap
	uint64_t start_sample = prev_length * MipMapScaleFactor;
	uint64_t end_sample = m0.length * MipMapScaleFactor;

	const int32_t start_sample_delta = p_start_sample - start_sample;

	#ifndef NDEBUG
	if (start_sample_delta < 0) {
		qDebug() << "ERROR: append_payload_to_mipmap() encountered start sample offset:" <<
			start_sample << p_start_sample << "; can't adjust by" << start_sample_delta <<
			"samples";
	}
	if (end_sample > p_end_sample) {
		qDebug() << "ERROR: append_payload_to_mipmap() encountered out-of-bounds condition:" <<
			end_sample << p_end_sample;
		return;
	}
	#else
	(void)p_end_sample;
	#endif

	const uint8_t* payload_ptr = payload;
	if (start_sample_delta > 0)
		payload_ptr += start_sample_delta * unit_size_;

	for (uint64_t i = start_sample; i < end_sample;) {
		// Accumulate transitions which have occurred in this sample
		accumulator = 0;
		diff_counter = MipMapScaleFactor;
		while (diff_counter-- > 0) {
			const uint64_t sample = unpack_sample(payload_ptr);
			accumulator |= last_append_sample_ ^ sample;
			last_append_sample_ = sample;
			i++;
			payload_ptr += unit_size_;
		}

		pack_sample(dest_ptr, accumulator);
		dest_ptr += unit_size_;
	}

	// Compute higher level mipmaps
	for (unsigned int level = 1; level < ScaleStepCount; level++) {
		MipMapLevel &m = mip_map_[level];
		const MipMapLevel &ml = mip_map_[level - 1];

		// Expand the data buffer to fit the new samples
		prev_length = m.length;
		m.length = ml.length / MipMapScaleFactor;

		// Break off if there are no more samples to be computed
		if (m.length == prev_length)
			break;

		reallocate_mipmap_level(m);

		// Subsample the lower level
		const uint8_t* src_ptr = (uint8_t*)ml.data +
			unit_size_ * prev_length * MipMapScaleFactor;
		const uint8_t *const end_dest_ptr =
			(uint8_t*)m.data + unit_size_ * m.length;

		for (dest_ptr = (uint8_t*)m.data +
				unit_size_ * prev_length;
				dest_ptr < end_dest_ptr;
				dest_ptr += unit_size_) {
			accumulator = 0;
			diff_counter = MipMapScaleFactor;
			while (diff_counter-- > 0) {
				accumulator |= unpack_sample(src_ptr);
				src_ptr += unit_size_;
			}

			pack_sample(dest_ptr, accumulator);
		}
	}
}

uint64_t LogicSegment::get_unpacked_sample(uint64_t index)
{
	assert(index < sample_count_);

	assert(unit_size_ <= 8);  // 8 * 8 = 64 channels
	uint8_t data[8];

	get_raw_samples(index, 1, data);
	uint64_t sample = unpack_sample(data);

	return sample;
}

void LogicSegment::get_subsampled_edges(
	vector<EdgePair> &edges,
	uint64_t start, uint64_t end,
	float min_length, int sig_index)
{
	uint64_t index = start;
	unsigned int level;
	bool last_sample;
	bool fast_forward;

	assert(start <= end);
	assert(min_length > 0);
	assert(sig_index >= 0);
	assert(sig_index < 64);

	lock_guard<recursive_mutex> lock(mutex_);

	// Make sure we only process as many samples as we have
	if (end > get_sample_count())
		end = get_sample_count();

	const uint64_t block_length = (uint64_t)max(min_length, 1.0f);
	const unsigned int min_level = max((int)floorf(logf(min_length) /
		LogMipMapScaleFactor) - 1, 0);
	const uint64_t sig_mask = 1ULL << sig_index;

	// Store the initial state
	last_sample = (get_unpacked_sample(start) & sig_mask) != 0;
	edges.emplace_back(index++, last_sample);

	while (index + block_length <= end) {
		//----- Continue to search -----//
		level = min_level;

		// We cannot fast-forward if there is no mip-map data at
		// the minimum level.
		fast_forward = (mip_map_[level].data != nullptr);

		if (min_length < MipMapScaleFactor) {
			// Search individual samples up to the beginning of
			// the next first level mip map block
			const uint64_t final_index = min(end, pow2_ceil(index, MipMapScalePower));

			for (; index < final_index &&
					(index & ~((uint64_t)(~0) << MipMapScalePower)) != 0;
					index++) {

				const bool sample = (get_unpacked_sample(index) & sig_mask) != 0;

				// If there was a change we cannot fast forward
				if (sample != last_sample) {
					fast_forward = false;
					break;
				}
			}
		} else {
			// If resolution is less than a mip map block,
			// round up to the beginning of the mip-map block
			// for this level of detail
			const int min_level_scale_power = (level + 1) * MipMapScalePower;
			index = pow2_ceil(index, min_level_scale_power);
			if (index >= end)
				break;

			// We can fast forward only if there was no change
			const bool sample = (get_unpacked_sample(index) & sig_mask) != 0;
			if (last_sample != sample)
				fast_forward = false;
		}

		if (fast_forward) {

			// Fast forward: This involves zooming out to higher
			// levels of the mip map searching for changes, then
			// zooming in on them to find the point where the edge
			// begins.

			// Slide right and zoom out at the beginnings of mip-map
			// blocks until we encounter a change
			while (true) {
				const int level_scale_power = (level + 1) * MipMapScalePower;
				const uint64_t offset = index >> level_scale_power;

				// Check if we reached the last block at this
				// level, or if there was a change in this block
				if (offset >= mip_map_[level].length ||
					(get_subsample(level, offset) &	sig_mask))
					break;

				if ((offset & ~((uint64_t)(~0) << MipMapScalePower)) == 0) {
					// If we are now at the beginning of a
					// higher level mip-map block ascend one
					// level
					if ((level + 1 >= ScaleStepCount) || (!mip_map_[level + 1].data))
						break;

					level++;
				} else {
					// Slide right to the beginning of the
					// next mip map block
					index = pow2_ceil(index + 1, level_scale_power);
				}
			}

			// Zoom in, and slide right until we encounter a change,
			// and repeat until we reach min_level
			while (true) {
				assert(mip_map_[level].data);

				const int level_scale_power = (level + 1) * MipMapScalePower;
				const uint64_t offset = index >> level_scale_power;

				// Check if we reached the last block at this
				// level, or if there was a change in this block
				if (offset >= mip_map_[level].length ||
						(get_subsample(level, offset) & sig_mask)) {
					// Zoom in unless we reached the minimum
					// zoom
					if (level == min_level)
						break;

					level--;
				} else {
					// Slide right to the beginning of the
					// next mip map block
					index = pow2_ceil(index + 1, level_scale_power);
				}
			}

			// If individual samples within the limit of resolution,
			// do a linear search for the next transition within the
			// block
			if (min_length < MipMapScaleFactor) {
				for (; index < end; index++) {
					const bool sample = (get_unpacked_sample(index) & sig_mask) != 0;
					if (sample != last_sample)
						break;
				}
			}
		}

		//----- Store the edge -----//

		// Take the last sample of the quanization block
		const int64_t final_index = index + block_length;
		if (index + block_length > end)
			break;

		// Store the final state
		const bool final_sample = (get_unpacked_sample(final_index - 1) & sig_mask) != 0;
		edges.emplace_back(index, final_sample);

		index = final_index;
		last_sample = final_sample;
	}

	// Add the final state
	const bool end_sample = get_unpacked_sample(end) & sig_mask;
	if (last_sample != end_sample)
		edges.emplace_back(end, end_sample);
	edges.emplace_back(end + 1, end_sample);
}

uint64_t LogicSegment::get_subsample(int level, uint64_t offset) const
{
	assert(level >= 0);
	assert(mip_map_[level].data);
	return unpack_sample((uint8_t*)mip_map_[level].data +
		unit_size_ * offset);
}

uint64_t LogicSegment::pow2_ceil(uint64_t x, unsigned int power)
{
	const uint64_t p = 1 << power;
	return (x + p - 1) / p * p;
}

} // namespace data
} // namespace pv
