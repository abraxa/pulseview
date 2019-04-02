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

#ifndef PULSEVIEW_PV_DATA_LOGICSEGMENT_HPP
#define PULSEVIEW_PV_DATA_LOGICSEGMENT_HPP

#include "segment.hpp"

#include <utility>
#include <vector>

#include <QObject>

using std::pair;
using std::shared_ptr;
using std::vector;

namespace sigrok {
	class Logic;
}

namespace LogicSegmentTest {
struct Pow2;
struct Basic;
struct LargeData;
struct Pulses;
struct LongPulses;
}

namespace pv {
namespace data {

class Logic;


class LogicSegment : public Segment
{
	Q_OBJECT

public:
	struct __attribute__((packed)) Edge {
		Edge(uint64_t n, bool s) :
			sample_num(n), new_state(s) {}
		uint64_t sample_num;
		bool new_state;
	};

	struct RLEData {
		vector<Edge> edges;
	};

public:
	LogicSegment(pv::data::Logic& owner, uint32_t segment_id,
		unsigned int unit_size, uint64_t samplerate);

	virtual ~LogicSegment();

	void append_payload(shared_ptr<sigrok::Logic> logic);
	void append_payload(void *data, uint64_t data_size);

	void get_samples(int64_t start_sample, int64_t end_sample, uint8_t* dest) const;

	/**
	 * Parses a logic data segment to generate a list of transitions
	 * in a time interval to a given level of detail.
	 * @param[out] edges The vector to place the edges into.
	 * @param[in] start The start sample index.
	 * @param[in] end The end sample index.
	 * @param[in] samples_per_pixel Number of samples per pixel, used to compact overlapping edges
	 * can be resolved at this level of detail.
	 * @param[in] sig_index The index of the signal.
	 */
	void get_subsampled_edges(vector<Edge> &edges,
		uint64_t start, uint64_t end, uint32_t samples_per_pixel,
		uint32_t sig_index, bool first_change_only = false);

	void get_surrounding_edges(vector<Edge> &dest,
		uint64_t origin_sample, uint32_t sig_index);

private:
	void process_new_samples(void *data, uint64_t samples);

private:
	Logic& owner_;

	uint64_t prev_sample_value_;
	vector<RLEData> sub_signals_;

	friend struct LogicSegmentTest::Pow2;
	friend struct LogicSegmentTest::Basic;
	friend struct LogicSegmentTest::LargeData;
	friend struct LogicSegmentTest::Pulses;
	friend struct LogicSegmentTest::LongPulses;
};

} // namespace data
} // namespace pv

#endif // PULSEVIEW_PV_DATA_LOGICSEGMENT_HPP
