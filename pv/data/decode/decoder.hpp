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
 * along with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#ifndef PULSEVIEW_PV_DATA_DECODE_DECODER_HPP
#define PULSEVIEW_PV_DATA_DECODE_DECODER_HPP

#include <map>
#include <memory>
#include <set>
#include <vector>

#include <glib.h>

#include <pv/data/signalbase.hpp>
#include <pv/data/decode/row.hpp>

using std::map;
using std::string;
using std::vector;

struct srd_decoder;
struct srd_decoder_inst;
struct srd_channel;
struct srd_session;

namespace pv {

namespace data {

class Logic;
class SignalBase;

namespace decode {

class Decoder;

struct AnnotationClass
{
	size_t id;
	char* name;
	char* description;
	Row* row;
	bool visible;
};

struct DecodeChannel
{
	uint16_t id;     ///< Global numerical ID for the decode channels in the stack
	uint16_t bit_id; ///< Tells which bit within a sample represents this channel
	const bool is_optional;
	const pv::data::SignalBase *assigned_signal;
	const QString name, desc;
	int initial_pin_state;
	const shared_ptr<Decoder> decoder_;
	const srd_channel *pdch_;
};

struct DecoderLogicOutputChannel {
	DecoderLogicOutputChannel (QString id, QString desc) :
		id(id), desc(desc) {};
	QString id, desc;
};

struct DecodeBinaryClassInfo
{
	uint32_t bin_class_id;
	char* name;
	char* description;
};


class Decoder
{
public:
	Decoder(const srd_decoder *const dec);

	virtual ~Decoder();

	const srd_decoder* get_srd_decoder() const;

	const char* name() const;

	bool visible() const;
	void set_visible(bool visible);

	const vector<DecodeChannel*>& channels() const;
	void set_channels(vector<DecodeChannel*> channels);

	const map<string, GVariant*>& options() const;

	void set_option(const char *id, GVariant *value);

	void apply_all_options();

	bool have_required_channels() const;

	srd_decoder_inst* create_decoder_inst(srd_session *session);
	void invalidate_decoder_inst();

	vector<Row*> get_rows();
	Row* get_row_by_id(size_t id);

	vector<const AnnotationClass*> ann_classes() const;
	vector<AnnotationClass*> ann_classes();
	AnnotationClass* get_ann_class_by_id(size_t id);
	const AnnotationClass* get_ann_class_by_id(size_t id) const;

	uint32_t get_binary_class_count() const;
	const DecodeBinaryClassInfo* get_binary_class(uint32_t id) const;

	bool has_logic_output() const;
	const vector<DecoderLogicOutputChannel> logic_output_channels() const;

private:
	const srd_decoder* const srd_decoder_;

	bool visible_;

	vector<DecodeChannel*> channels_;
	vector<Row> rows_;
	vector<AnnotationClass> ann_classes_;
	vector<DecodeBinaryClassInfo> bin_classes_;
	map<string, GVariant*> options_;
	srd_decoder_inst *decoder_inst_;
};

} // namespace decode
} // namespace data
} // namespace pv

#endif // PULSEVIEW_PV_DATA_DECODE_DECODER_HPP
