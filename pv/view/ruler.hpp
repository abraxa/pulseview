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

#ifndef PULSEVIEW_PV_VIEW_RULER_H
#define PULSEVIEW_PV_VIEW_RULER_H

#include <memory>

#include "marginwidget.hpp"

namespace pv {
namespace view {

class Ruler : public MarginWidget
{
	Q_OBJECT

private:
	static const int RulerHeight;
	static const int MinorTickSubdivision;

	static const int HoverArrowSize;

public:
	Ruler(View &parent);

public:
	QSize sizeHint() const;

private:
	void paintEvent(QPaintEvent *event);

private:
	/**
	 * Draw a hover arrow under the cursor position.
	 */
	void draw_hover_mark(QPainter &p);

private Q_SLOTS:
	void hover_point_changed();
};

} // namespace view
} // namespace pv

#endif // PULSEVIEW_PV_VIEW_RULER_H
