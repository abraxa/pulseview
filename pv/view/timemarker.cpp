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
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA
 */

#include <algorithm>

#include <extdef.h>

#include "timemarker.hpp"

#include "view.hpp"

#include <QFormLayout>
#include <QPainter>

#include <pv/util.hpp>
#include <pv/widgets/popup.hpp>

using std::max;
using std::min;

namespace pv {
namespace view {

const int TimeMarker::ArrowSize = 4;
const int TimeMarker::Offset = 1;

TimeMarker::TimeMarker(View &view, const QColor &colour, double time) :
	view_(view),
	colour_(colour),
	time_(time),
	value_action_(NULL),
	value_widget_(NULL),
	updating_value_widget_(false)
{
}

double TimeMarker::time() const
{
	return time_;
}

float TimeMarker::get_x() const
{
	return (time_ - view_.offset()) / view_.scale();
}

QPoint TimeMarker::point() const
{
	return QPoint(get_x(), 0);
}

void TimeMarker::set_time(double time)
{
	time_ = time;

	if (value_widget_) {
		updating_value_widget_ = true;
		value_widget_->setValue(time);
		updating_value_widget_ = false;
	}

	time_changed();
}

void TimeMarker::paint(QPainter &p, const QRect &rect)
{
	const float x = get_x();
	p.setPen(colour_.darker());
	p.drawLine(QPointF(x, rect.top()), QPointF(x, rect.bottom()));
}

void TimeMarker::paint_label(QPainter &p, const QRect &rect)
{
	const qreal x = (time_ - view_.offset()) / view_.scale();
	const QRectF r(get_label_rect(rect));

	const QPointF points[] = {
		r.topLeft(),
		r.bottomLeft(),
		QPointF(max(r.left(), x - ArrowSize), r.bottom()),
		QPointF(x, rect.bottom()),
		QPointF(min(r.right(), x + ArrowSize), r.bottom()),
		r.bottomRight(),
		r.topRight()
	};

	const QPointF highlight_points[] = {
		QPointF(r.left() + 1, r.top() + 1),
		QPointF(r.left() + 1, r.bottom() - 1),
		QPointF(max(r.left() + 1, x - ArrowSize), r.bottom() - 1),
		QPointF(min(max(r.left() + 1, x), r.right() - 1),
			rect.bottom() - 1),
		QPointF(min(r.right() - 1, x + ArrowSize), r.bottom() - 1),
		QPointF(r.right() - 1, r.bottom() - 1),
		QPointF(r.right() - 1, r.top() + 1),
	};

	if (selected()) {
		p.setPen(highlight_pen());
		p.setBrush(Qt::transparent);
		p.drawPolygon(points, countof(points));
	}

	p.setPen(Qt::transparent);
	p.setBrush(colour_);
	p.drawPolygon(points, countof(points));

	p.setPen(colour_.lighter());
	p.setBrush(Qt::transparent);
	p.drawPolygon(highlight_points, countof(highlight_points));

	p.setPen(colour_.darker());
	p.setBrush(Qt::transparent);
	p.drawPolygon(points, countof(points));

	p.setPen(select_text_colour(colour_));
	p.drawText(r, Qt::AlignCenter | Qt::AlignVCenter,
		pv::util::format_time(time_, view_.tick_prefix(), 2));
}

pv::widgets::Popup* TimeMarker::create_popup(QWidget *parent)
{
	using pv::widgets::Popup;

	Popup *const popup = new Popup(parent);
	QFormLayout *const form = new QFormLayout(popup);
	popup->setLayout(form);

	value_widget_ = new QDoubleSpinBox(parent);
	value_widget_->setDecimals(9);
	value_widget_->setSuffix("s");
	value_widget_->setSingleStep(1e-6);
	value_widget_->setRange(-1.0e9, 1.0e9);
	value_widget_->setValue(time_);

	connect(value_widget_, SIGNAL(valueChanged(double)),
		this, SLOT(on_value_changed(double)));

	form->addRow(tr("Time"), value_widget_);

	return popup;
}

void TimeMarker::on_value_changed(double value)
{
	if (!updating_value_widget_) {
		time_ = value;
		time_changed();
	}
}

} // namespace view
} // namespace pv
