/*
 * This file is part of the PulseView project.
 *
 * Copyright (C) 2012 Joel Holdsworth <joel@airwebreathe.org.uk>
 * Copyright (C) 2016 Soeren Apel <soeren@apelpie.net>
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

#ifdef ENABLE_DECODE
#include <libsigrokdecode/libsigrokdecode.h>
#endif

#include <libsigrokcxx/libsigrokcxx.hpp>

#include "pv/session.hpp"
#include "pv/util.hpp"
#include "pv/data/segment.hpp"

using std::shared_ptr;

namespace pv {
namespace views {

const char* ViewTypeNames[ViewTypeCount] = {
	"Trace View",
#ifdef ENABLE_DECODE
	"Binary Decoder Output View"
#endif
};

const int ViewBase::MaxViewAutoUpdateRate = 25; // No more than 25 Hz

ViewBase::ViewBase(Session &session, bool is_main_view, QMainWindow *parent) :
	// Note: Place defaults in ViewBase::reset_view_state(), not here
	QWidget(parent),
	session_(session),
	is_main_view_(is_main_view)
{
	(void)parent;

	connect(&session_, SIGNAL(signals_changed()),
		this, SLOT(signals_changed()));
	connect(&session_, SIGNAL(capture_state_changed(int)),
		this, SLOT(capture_state_updated(int)));
	connect(&session_, SIGNAL(new_segment(int)),
		this, SLOT(on_new_segment(int)));

	connect(&delayed_view_updater_, SIGNAL(timeout()),
		this, SLOT(perform_delayed_view_update()));
	delayed_view_updater_.setSingleShot(true);
	delayed_view_updater_.setInterval(1000 / MaxViewAutoUpdateRate);
}

bool ViewBase::is_main_view() const
{
	return is_main_view_;
}

void ViewBase::reset_view_state()
{
	current_segment_ = 0;
}

Session& ViewBase::session()
{
	return session_;
}

const Session& ViewBase::session() const
{
	return session_;
}

void ViewBase::clear_signals()
{
	clear_signalbases();
}

vector< shared_ptr<data::SignalBase> > ViewBase::signalbases() const
{
	return signalbases_;
}

void ViewBase::clear_signalbases()
{
	for (const shared_ptr<data::SignalBase>& signalbase : signalbases_) {
		disconnect(signalbase.get(), SIGNAL(samples_cleared()),
			this, SLOT(on_data_updated()));
		disconnect(signalbase.get(), SIGNAL(samples_added(uint64_t, uint64_t, uint64_t)),
			this, SLOT(on_samples_added(uint64_t, uint64_t, uint64_t)));
	}

	signalbases_.clear();
}

void ViewBase::add_signalbase(const shared_ptr<data::SignalBase> signalbase)
{
	signalbases_.push_back(signalbase);

	connect(signalbase.get(), SIGNAL(samples_cleared()),
		this, SLOT(on_data_updated()));
	connect(signalbase.get(), SIGNAL(samples_added(uint64_t, uint64_t, uint64_t)),
		this, SLOT(on_samples_added(uint64_t, uint64_t, uint64_t)));
}

#ifdef ENABLE_DECODE
void ViewBase::clear_decode_signals()
{
	decode_signals_.clear();
}

void ViewBase::add_decode_signal(shared_ptr<data::DecodeSignal> signal)
{
	decode_signals_.push_back(signal);
}

void ViewBase::remove_decode_signal(shared_ptr<data::DecodeSignal> signal)
{
	decode_signals_.erase(std::remove_if(
		decode_signals_.begin(), decode_signals_.end(),
		[&](shared_ptr<data::DecodeSignal> s) { return s == signal; }),
		decode_signals_.end());
}
#endif

void ViewBase::save_settings(QSettings &settings) const
{
	(void)settings;
}

void ViewBase::restore_settings(QSettings &settings)
{
	(void)settings;
}

void ViewBase::trigger_event(int segment_id, util::Timestamp location)
{
	(void)segment_id;
	(void)location;
}

void ViewBase::signals_changed()
{
}

void ViewBase::on_new_segment(int new_segment_id)
{
	(void)new_segment_id;
}

void ViewBase::on_segment_completed(int new_segment_id)
{
	(void)new_segment_id;
}

void ViewBase::capture_state_updated(int state)
{
	(void)state;
}

void ViewBase::perform_delayed_view_update()
{
}

void ViewBase::on_samples_added(uint64_t segment_id, uint64_t start_sample,
	uint64_t end_sample)
{
	(void)start_sample;
	(void)end_sample;

	if (segment_id != current_segment_)
		return;

	if (!delayed_view_updater_.isActive())
		delayed_view_updater_.start();
}

void ViewBase::on_data_updated()
{
	if (!delayed_view_updater_.isActive())
		delayed_view_updater_.start();
}

}  // namespace views
} // namespace pv
