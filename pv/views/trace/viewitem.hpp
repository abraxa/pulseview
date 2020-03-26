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

#ifndef PULSEVIEW_PV_VIEWITEM_HPP
#define PULSEVIEW_PV_VIEWITEM_HPP

#include <list>

#include <QMouseEvent>
#include <QPen>
#include <QPoint>

#include "pv/views/trace/viewitempaintparams.hpp"

class QAction;
class QMenu;
class QWidget;

namespace pv {

namespace widgets {
class Popup;
}

namespace views {
namespace trace {

class ViewItemOwner;

class ViewItem : public QObject
{
	Q_OBJECT

public:
	static const QSizeF LabelPadding;
	static const int HighlightRadius;

public:
	ViewItem();

	/**
	 * Returns true if the item is visible and enabled.
	 */
	virtual bool enabled() const = 0;

	/**
	 * Returns true if the item may be selected.
	 */
	virtual bool is_selectable(QPoint pos) const;

	/**
	 * Returns true if the item has been selected by the user.
	 */
	bool selected() const;

	/**
	 * Selects or deselects the signal.
	 */
	virtual void select(bool select = true);

	/**
	 * Returns true if the item may be dragged/moved.
	 */
	virtual bool is_draggable(QPoint pos) const;

	/**
	 * Returns true if the item is being dragged.
	 */
	bool dragging() const;

	/**
	 * Sets this item into the dragged state.
	 */
	void drag();

	/**
	 * Sets this item into the un-dragged state.
	 */
	virtual void drag_release();

	/**
	 * Drags the item to a delta relative to the drag point.
	 * @param delta the offset from the drag point.
	 */
	virtual void drag_by(const QPoint &delta) = 0;

	/**
	 * Get the drag point.
	 * @param rect the rectangle of the widget area.
	 */
	virtual QPoint drag_point(const QRect &rect) const = 0;

	/**
	 * Computes the outline rectangle of a label.
	 * @param rect the rectangle of the header area.
	 * @return Returns the rectangle of the signal label.
	 * @remarks The default implementation returns an empty rectangle.
	 */
	virtual QRectF label_rect(const QRectF &rect) const;

	/**
	 * Computes the outline rectangle of the viewport hit-box.
	 * @param rect the rectangle of the viewport area.
	 * @return Returns the rectangle of the hit-box.
	 * @remarks The default implementation returns an empty hit-box.
	 */
	virtual QRectF hit_box_rect(const ViewItemPaintParams &pp) const;

	/**
	 * Paints the signal label.
	 * @param p the QPainter to paint into.
	 * @param rect the rectangle of the header area.
	 * @param hover true if the label is being hovered over by the mouse.
	 */
	virtual void paint_label(QPainter &p, const QRect &rect, bool hover);

	/**
	 * Paints the background layer of the item with a QPainter
	 * @param p the QPainter to paint into.
	 * @param pp the painting parameters object to paint with.
	 */
	virtual void paint_back(QPainter &p, ViewItemPaintParams &pp);

	/**
	 * Paints the mid-layer of the item with a QPainter
	 * @param p the QPainter to paint into.
	 * @param pp the painting parameters object to paint with.
	 */
	virtual void paint_mid(QPainter &p, ViewItemPaintParams &pp);

	/**
	 * Paints the foreground layer of the item with a QPainter
	 * @param p the QPainter to paint into.
	 * @param pp the painting parameters object to paint with.
	 */
	virtual void paint_fore(QPainter &p, ViewItemPaintParams &pp);

	/**
	 * Gets the text color.
	 * @remarks This color is computed by comparing the lightness
	 * of the trace color against a threshold to determine whether
	 * white or black would be more visible.
	 */
	static QColor select_text_color(QColor background);

public:
	virtual QMenu* create_header_context_menu(QWidget *parent);

	virtual QMenu* create_view_context_menu(QWidget *parent, QPoint &click_pos);

	virtual pv::widgets::Popup* create_popup(QWidget *parent);

	virtual void delete_pressed();

	virtual void hover_point_changed(const QPoint &hp);

	/**
	 * Handles left mouse button press events.
	 * @param event the mouse event that triggered this handler.
	 */
	virtual void mouse_left_press_event(const QMouseEvent* event);

protected:
	static QPen highlight_pen();

protected:
	QWidget *context_parent_;
	QPoint drag_point_;

private:
	bool selected_;
};

} // namespace trace
} // namespace views
} // namespace pv

#endif // PULSEVIEW_PV_VIEWITEM_HPP
