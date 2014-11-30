/*
 * This file is part of the PulseView project.
 *
 * Copyright (C) 2013 Joel Holdsworth <joel@airwebreathe.org.uk>
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
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA
 */

#include <extdef.h>

#include <stdint.h>

#include <boost/test/unit_test.hpp>

#include <pv/data/analogsegment.hpp>

using pv::data::AnalogSegment;

BOOST_AUTO_TEST_SUITE(AnalogSegmentTest)

void push_analog(AnalogSegment &s, unsigned int num_samples,
	float value)
{
	float *const data = new float[num_samples];
	for (unsigned int i = 0; i < num_samples; i++)
		data[i] = value;

	s.append_interleaved_samples(data, num_samples, 1);
	delete[] data;
}

BOOST_AUTO_TEST_CASE(Basic)
{
	// Create an empty AnalogSegment object
	AnalogSegment s;

	//----- Test AnalogSegment::push_analog -----//

	BOOST_CHECK(s.get_sample_count() == 0);
	for (unsigned int i = 0; i < AnalogSegment::ScaleStepCount; i++)
	{
		const AnalogSegment::Envelope &m = s.envelope_levels_[i];
		BOOST_CHECK_EQUAL(m.length, 0);
		BOOST_CHECK_EQUAL(m.data_length, 0);
		BOOST_CHECK(m.samples == NULL);
	}

	// Push 8 samples of all zeros
	push_analog(s, 8, 0.0f);

	BOOST_CHECK(s.get_sample_count() == 8);

	// There should not be enough samples to have a single mip map sample
	for (unsigned int i = 0; i < AnalogSegment::ScaleStepCount; i++)
	{
		const AnalogSegment::Envelope &m = s.envelope_levels_[i];
		BOOST_CHECK_EQUAL(m.length, 0);
		BOOST_CHECK_EQUAL(m.data_length, 0);
		BOOST_CHECK(m.samples == NULL);
	}

	// Push 8 samples of 1.0s to bring the total up to 16
	push_analog(s, 8, 1.0f);

	// There should now be enough data for exactly one sample
	// in mip map level 0, and that sample should be 0
	const AnalogSegment::Envelope &e0 = s.envelope_levels_[0];
	BOOST_CHECK_EQUAL(e0.length, 1);
	BOOST_CHECK_EQUAL(e0.data_length, AnalogSegment::EnvelopeDataUnit);
	BOOST_REQUIRE(e0.samples != NULL);
	BOOST_CHECK_EQUAL(e0.samples[0].min, 0.0f);
	BOOST_CHECK_EQUAL(e0.samples[0].max, 1.0f);

	// The higher levels should still be empty
	for (unsigned int i = 1; i < AnalogSegment::ScaleStepCount; i++)
	{
		const AnalogSegment::Envelope &m = s.envelope_levels_[i];
		BOOST_CHECK_EQUAL(m.length, 0);
		BOOST_CHECK_EQUAL(m.data_length, 0);
		BOOST_CHECK(m.samples == NULL);
	}

	// Push 240 samples of all zeros to bring the total up to 256
	push_analog(s, 240, -1.0f);

	BOOST_CHECK_EQUAL(e0.length, 16);
	BOOST_CHECK_EQUAL(e0.data_length, AnalogSegment::EnvelopeDataUnit);

	for (unsigned int i = 1; i < e0.length; i++) {
		BOOST_CHECK_EQUAL(e0.samples[i].min, -1.0f);
		BOOST_CHECK_EQUAL(e0.samples[i].max, -1.0f);
	}

	const AnalogSegment::Envelope &e1 = s.envelope_levels_[1];
	BOOST_CHECK_EQUAL(e1.length, 1);
	BOOST_CHECK_EQUAL(e1.data_length, AnalogSegment::EnvelopeDataUnit);
	BOOST_REQUIRE(e1.samples != NULL);
	BOOST_CHECK_EQUAL(e1.samples[0].min, -1.0f);
	BOOST_CHECK_EQUAL(e1.samples[0].max, 1.0f);
}

BOOST_AUTO_TEST_SUITE_END()
