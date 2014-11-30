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

#include "cursorpair.hpp"

#include "view.hpp"
#include "pv/util.hpp"

#include <cassert>
#include <algorithm>

using std::max;
using std::make_pair;
using std::min;
using std::shared_ptr;
using std::pair;

namespace pv {
namespace view {

const int CursorPair::DeltaPadding = 8;

CursorPair::CursorPair(View &view) :
	first_(new Cursor(view, 0.0)),
	second_(new Cursor(view, 1.0)),
	view_(view)
{
}

shared_ptr<Cursor> CursorPair::first() const
{
	return first_;
}

shared_ptr<Cursor> CursorPair::second() const
{
	return second_;
}

QRectF CursorPair::get_label_rect(const QRect &rect) const
{
	const QSizeF label_size(
		text_size_.width() + View::LabelPadding.width() * 2,
		text_size_.height() + View::LabelPadding.height() * 2);
	const pair<float, float> offsets(get_cursor_offsets());
	const pair<float, float> normal_offsets(
		(offsets.first < offsets.second) ? offsets :
		make_pair(offsets.second, offsets.first));

	const float height = label_size.height();
	const float left = max(normal_offsets.first + DeltaPadding, -height);
	const float right = min(normal_offsets.second - DeltaPadding,
		(float)rect.width() + height);

	return QRectF(left, rect.height() - label_size.height() -
		Cursor::ArrowSize - Cursor::Offset - 0.5f,
		right - left, height);
}

void CursorPair::draw_markers(QPainter &p, const QRect &rect)
{
	assert(first_);
	assert(second_);

	const unsigned int prefix = view_.tick_prefix();

	compute_text_size(p, prefix);
	QRectF delta_rect(get_label_rect(rect));

	const int radius = delta_rect.height() / 2;
	const QRectF text_rect(delta_rect.intersected(
		rect).adjusted(radius, 0, -radius, 0));
	if(text_rect.width() >= text_size_.width())
	{
		const int highlight_radius = delta_rect.height() / 2 - 2;

		p.setBrush(Cursor::FillColour);
		p.setPen(Cursor::LineColour);
		p.drawRoundedRect(delta_rect, radius, radius);

		delta_rect.adjust(1, 1, -1, -1);
		p.setPen(Cursor::HighlightColour);
		p.drawRoundedRect(delta_rect, highlight_radius, highlight_radius);

		p.setPen(Cursor::TextColour);
		p.drawText(text_rect, Qt::AlignCenter | Qt::AlignVCenter,
			pv::util::format_time(second_->time() - first_->time(), prefix, 2));
	}

	// Paint the cursor markers
	first_->paint_label(p, rect);
	second_->paint_label(p, rect);
}

void CursorPair::draw_viewport_background(QPainter &p,
	const QRect &rect)
{
	p.setPen(Qt::NoPen);
	p.setBrush(QBrush(View::CursorAreaColour));

	const pair<float, float> offsets(get_cursor_offsets());
	const int l = (int)max(min(
		offsets.first, offsets.second), 0.0f);
	const int r = (int)min(max(
		offsets.first, offsets.second), (float)rect.width());

	p.drawRect(l, 0, r - l, rect.height());
}

void CursorPair::draw_viewport_foreground(QPainter &p,
	const QRect &rect)
{
	assert(first_);
	assert(second_);

	first_->paint(p, rect);
	second_->paint(p, rect);
}

void CursorPair::compute_text_size(QPainter &p, unsigned int prefix)
{
	assert(first_);
	assert(second_);

	text_size_ = p.boundingRect(QRectF(), 0, pv::util::format_time(
		second_->time() - first_->time(), prefix, 2)).size();
}

pair<float, float> CursorPair::get_cursor_offsets() const
{
	assert(first_);
	assert(second_);

	return pair<float, float>(
		(first_->time() - view_.offset()) / view_.scale(),
		(second_->time() - view_.offset()) / view_.scale());
}

} // namespace view
} // namespace pv
