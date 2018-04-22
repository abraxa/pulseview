/*
 * This file is part of the PulseView project.
 *
 * Copyright (C) 2017 Soeren Apel <soeren@apelpie.net>
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

#include "segment.hpp"

#include <cassert>
#include <cstddef>
#include <cstdlib>
#include <cstring>

#include <QDebug>

using std::lock_guard;
using std::min;
using std::out_of_range;
using std::recursive_mutex;
using std::size_t;

namespace pv {
namespace data {

const uint64_t Segment::MaxChunkSize = 10 * 1024 * 1024;  /* 10MiB */
const uint64_t Segment::NotifyBlockSize = 2 * MaxChunkSize;
const int Segment::CompressionLevel = 3;

Segment::Segment(SignalData& owner, uint32_t segment_id, uint64_t samplerate,
	unsigned int unit_size) :
	owner_(owner),
	segment_id_(segment_id),
#ifdef ENABLE_ZSTD
	input_chunk_(nullptr),
	output_chunk_(nullptr),
	output_chunk_num_(INT_MAX),
#endif
	sample_count_(0),
	first_new_sample_(0),
	new_samples_(0),
	start_time_(0),
	samplerate_(samplerate),
	unit_size_(unit_size),
	is_complete_(false)
{
	lock_guard<recursive_mutex> lock(mutex_);
	assert(unit_size_ > 0);

	// Determine the number of bytes we can fit in one chunk
	// without exceeding MaxChunkSize
	chunk_size_ = min(MaxChunkSize, (MaxChunkSize / unit_size_) * unit_size_);

#ifdef ENABLE_ZSTD
	// Create the input chunk
	input_chunk_ = new uint8_t[chunk_size_];
#else
	// Create the initial chunk
	current_chunk_ = new uint8_t[chunk_size_];
	data_chunks_.push_back(current_chunk_);
#endif
	used_samples_ = 0;
	unused_samples_ = chunk_size_ / unit_size_;
}

Segment::~Segment()
{
	lock_guard<recursive_mutex> lock(mutex_);

#ifdef ENABLE_ZSTD
	if (input_chunk_)
		delete[] input_chunk_;
	if (output_chunk_)
		delete[] output_chunk_;
#else
	for (uint8_t* chunk : data_chunks_)
		delete[] chunk;
#endif
}

uint64_t Segment::get_sample_count() const
{
	lock_guard<recursive_mutex> lock(mutex_);
	return sample_count_;
}

const pv::util::Timestamp& Segment::start_time() const
{
	return start_time_;
}

double Segment::samplerate() const
{
	return samplerate_;
}

void Segment::set_samplerate(double samplerate)
{
	samplerate_ = samplerate;
}

unsigned int Segment::unit_size() const
{
	return unit_size_;
}

uint32_t Segment::segment_id() const
{
	return segment_id_;
}

void Segment::set_complete()
{
	is_complete_ = true;

#ifdef ENABLE_ZSTD
	// Finalize input operations so that the segment can no longer grow in size
	if (used_samples_ > 0) {
		compress_input_chunk();
		used_samples_ = 0;
	}

	if (input_chunk_) {
		input_chunk_ = nullptr;
		input_chunk_num_ = INT_MAX; // Prevent append_samples() from accessing input_chunk_
		unused_samples_ = 0;
		delete[] input_chunk_;
	}
#endif

	if (new_samples_ > 0)
		signal_new_samples();
}

bool Segment::is_complete() const
{
	return is_complete_;
}

void Segment::signal_new_samples()
{
	if (new_samples_ == 0)
		return;

	owner_.signal_samples_added(this, first_new_sample_,
		first_new_sample_ + new_samples_);

	first_new_sample_ = first_new_sample_ + new_samples_ + 1;
	new_samples_ = 0;
}

#ifdef ENABLE_ZSTD
void Segment::compress_input_chunk()
{
	size_t result = 0;

	compressed_chunks_.emplace_back(ZSTD_compressBound(chunk_size_));
	vector<uint8_t>& compressed_chunk = compressed_chunks_.back();

	result = ZSTD_compress(compressed_chunk.data(), compressed_chunk.capacity(),
		(void*)input_chunk_, used_samples_ * unit_size_, Segment::CompressionLevel);

	if (ZSTD_isError(result)) {
		qDebug() << "ERROR: ZSTD_compress for segment" << segment_id_ << ":" <<
			QString::fromUtf8(ZSTD_getErrorName(result));
	} else {
		// Result contains number of written bytes
		compressed_chunk.resize(result);
		compressed_chunk.shrink_to_fit();
	}
}

void Segment::decompress_chunk(uint64_t chunk_num, uint8_t *dest) const
{
	size_t result = 0;

	try {
		const vector<uint8_t>& compressed_chunk = compressed_chunks_.at(chunk_num);
		result = ZSTD_decompress((void*)dest, chunk_size_ * unit_size_,
			compressed_chunk.data(), compressed_chunk.size());

		if (ZSTD_isError(result)) {
			qDebug() << "ERROR: ZSTD_decompress for segment" << segment_id_ << ":" <<
				QString::fromUtf8(ZSTD_getErrorName(result));
		}

	} catch (out_of_range&) {
		qDebug() << "ERROR: Chunk" << chunk_num << "out of range for segment" <<
			segment_id_ << "; max:" << (compressed_chunks_.size() - 1);
		return;
	}
}
#endif

void Segment::append_samples(void* data, uint64_t samples)
{
	lock_guard<recursive_mutex> lock(mutex_);

	if (unused_samples_ == 0) {
		qDebug() << "WARNING: Failed to append" << samples <<
			"samples to segment" << segment_id_;
		return;
	}

	const uint8_t* data_byte_ptr = (uint8_t*)data;
	uint64_t remaining_samples = samples;
	uint64_t data_offset = 0;

	do {
		uint64_t copy_count = 0;

		if (remaining_samples <= unused_samples_) {
			// All samples fit into the current chunk
			copy_count = remaining_samples;
		} else {
			// Only a part of the samples fit, fill up current chunk
			copy_count = unused_samples_;
		}

#ifdef ENABLE_ZSTD
		const uint8_t* dest = &(input_chunk_[used_samples_ * unit_size_]);
#else
		const uint8_t* dest = &(current_chunk_[used_samples_ * unit_size_]);
#endif
		const uint8_t* src = &(data_byte_ptr[data_offset]);
		memcpy((void*)dest, (void*)src, (copy_count * unit_size_));

		used_samples_ += copy_count;
		unused_samples_ -= copy_count;
		remaining_samples -= copy_count;
		data_offset += (copy_count * unit_size_);

		if (unused_samples_ == 0) {
#ifdef ENABLE_ZSTD
			// Compress the chunk's data and file the compressed data away
			compress_input_chunk();
			input_chunk_num_++;
#else
			// If we're out of memory, this will throw std::bad_alloc
			current_chunk_ = new uint8_t[chunk_size_];
			data_chunks_.push_back(current_chunk_);
#endif
			used_samples_ = 0;
			unused_samples_ = chunk_size_ / unit_size_;
		}
	} while (remaining_samples > 0);

	sample_count_ += samples;
	new_samples_ += samples;

	if (new_samples_ >= NotifyBlockSize)
		signal_new_samples();
}

void Segment::get_raw_samples(uint64_t start, uint64_t count,
	uint8_t* dest)
{
	assert(start < sample_count_);
	assert(start + count <= sample_count_);
	assert(count > 0);
	assert(dest != nullptr);

	lock_guard<recursive_mutex> lock(mutex_);

	uint8_t* dest_ptr = dest;

#ifdef ENABLE_ZSTD
	if (!output_chunk_)
		output_chunk_ = new uint8_t[chunk_size_];
#endif

	uint64_t chunk_num = (start * unit_size_) / chunk_size_;
	uint64_t chunk_offs = (start * unit_size_) % chunk_size_;

	while (count > 0) {
		uint64_t copy_size = min(count * unit_size_, chunk_size_ - chunk_offs);

#ifdef ENABLE_ZSTD
		// Use input chunk if requested samples haven't been compressed yet
		// and only update output chunk data if requested chunk differs
		if (chunk_num == input_chunk_num_) {
			if (chunk_offs > used_samples_ * unit_size_)
				qDebug() << "WARNING: Accessing input chunk beyond valid data range";
			memcpy(dest_ptr, input_chunk_ + chunk_offs, copy_size);
		} else {
			if (chunk_num != output_chunk_num_) {
				decompress_chunk(chunk_num, output_chunk_);
				output_chunk_num_ = chunk_num;
			}
			memcpy(dest_ptr, output_chunk_ + chunk_offs, copy_size);
		}
#else
		const uint8_t* chunk = data_chunks_[chunk_num];
		memcpy(dest_ptr, chunk + chunk_offs, copy_size);
#endif

		dest_ptr += copy_size;
		count -= (copy_size / unit_size_);

		chunk_num++;
		chunk_offs = 0;
	}
}

} // namespace data
} // namespace pv
