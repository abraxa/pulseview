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

#ifndef PULSEVIEW_PV_VIEWS_TRACEVIEW_LOGICSIGNAL_HPP
#define PULSEVIEW_PV_VIEWS_TRACEVIEW_LOGICSIGNAL_HPP

#include <QCache>
#include <QColor>
#include <QDebug>
#include <QSpinBox>

#include "signal.hpp"

#include <memory>

using std::pair;
using std::shared_ptr;
using std::vector;

class QIcon;
class QToolBar;

namespace sigrok {
class TriggerMatchType;
}

namespace pv {

namespace devices {
class Device;
}

namespace data {
class Logic;
}

namespace views {
namespace trace {

class LogicSignal : public Signal
{
	Q_OBJECT

public:
	static const QColor EdgeColor;
	static const QColor HighColor;
	static const QColor LowColor;
	static const QColor SamplingPointColor;

	static const QColor SignalColors[10];

	static QColor TriggerMarkerBackgroundColor;
	static const int TriggerMarkerPadding;
	static const char* TriggerMarkerIcons[8];

	LogicSignal(pv::Session &session,
		shared_ptr<devices::Device> device,
		shared_ptr<data::SignalBase> base);

	virtual ~LogicSignal() = default;

	shared_ptr<pv::data::SignalData> data() const;

	shared_ptr<pv::data::Logic> logic_data() const;

	virtual void save_settings(QSettings &settings) const;
	virtual void restore_settings(QSettings &settings);

	/**
	 * Computes the vertical extents of the contents of this row item.
	 * @return A pair containing the minimum and maximum y-values.
	 */
	pair<int, int> v_extents() const;

	/**
	 * Paints the mid-layer of the signal with a QPainter
	 * @param p the QPainter to paint into.
	 * @param pp the painting parameters object to paint with..
	 */
	void paint_mid(QPainter &p, ViewItemPaintParams &pp);

	/**
	 * Paints the foreground layer of the signal with a QPainter
	 * @param p the QPainter to paint into.
	 * @param pp the painting parameters object to paint with.
	 */
	virtual void paint_fore(QPainter &p, ViewItemPaintParams &pp);

	/**
	 * Determines the closest level change (i.e. edge) to a given sample, which
	 * is useful for e.g. the "snap to edge" functionality.
	 *
	 * @param sample_pos Sample to use
	 * @return The changes left and right of the given position
	 */
	virtual vector<data::LogicSegment::EdgePair> get_nearest_level_changes(uint64_t sample_pos);

private:
	void paint_caps(QPainter &p, vector<data::LogicSegment::EdgePair> &edges,
		double samples_per_pixel, double pixels_offset,
		float x_offset, float low_offset, float high_offset, int64_t end_sample);

	shared_ptr<pv::data::LogicSegment> get_logic_segment_to_paint() const;

	void init_trigger_actions(QWidget *parent);

	const vector<int32_t> get_trigger_types() const;
	QAction* action_from_trigger_type(const sigrok::TriggerMatchType *type);
	const sigrok::TriggerMatchType* trigger_type_from_action(
		QAction *action);
	void populate_popup_form(QWidget *parent, QFormLayout *form);
	void modify_trigger();

	static const QIcon* get_icon(const char *path);
	static const QPixmap* get_pixmap(const char *path);

private Q_SLOTS:
	void on_setting_changed(const QString &key, const QVariant &value);

	void on_trigger();

	void on_signal_height_changed(int height);

private:
	int signal_height_;
	QColor high_fill_color_;
	bool show_sampling_points_, fill_high_areas_;

	shared_ptr<pv::devices::Device> device_;

	QSpinBox *signal_height_sb_;

	const sigrok::TriggerMatchType *trigger_match_;
	const vector<int32_t> trigger_types_;
	QToolBar *trigger_bar_;
	QAction *trigger_none_;
	QAction *trigger_rising_;
	QAction *trigger_high_;
	QAction *trigger_falling_;
	QAction *trigger_low_;
	QAction *trigger_change_;

	static QCache<QString, const QIcon> icon_cache_;
	static QCache<QString, const QPixmap> pixmap_cache_;
};

} // namespace trace
} // namespace views
} // namespace pv

#endif // PULSEVIEW_PV_VIEWS_TRACEVIEW_LOGICSIGNAL_HPP
