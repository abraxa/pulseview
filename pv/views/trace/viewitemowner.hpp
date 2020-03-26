/*
 * This file is part of the PulseView project.
 *
 * Copyright (C) 2014 Joel Holdsworth <joel@airwebreathe.org.uk>
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

#ifndef PULSEVIEW_PV_VIEWS_TRACEVIEW_VIEWITEMOWNER_HPP
#define PULSEVIEW_PV_VIEWS_TRACEVIEW_VIEWITEMOWNER_HPP

#include <memory>
#include <vector>

#include "pv/views/trace/viewitemiterator.hpp"

using std::dynamic_pointer_cast;
using std::shared_ptr;
using std::vector;

namespace pv {

class Session;

namespace views {
namespace trace {

class ViewItem;
class View;

class ViewItemOwner
{
public:
	typedef vector< shared_ptr<ViewItem> > item_list;
	typedef ViewItemIterator<ViewItemOwner, ViewItem> iterator;
	typedef ViewItemIterator<const ViewItemOwner, ViewItem> const_iterator;

public:
	/**
	 * Returns a list of row items owned by this object.
	 */
	virtual const item_list& child_items() const;

	/**
	 * Returns a depth-first iterator at the beginning of the child ViewItem
	 * tree.
	 */
	iterator begin();

	/**
	 * Returns a depth-first iterator at the end of the child ViewItem tree.
	 */
	iterator end();

	/**
	 * Returns a constant depth-first iterator at the beginning of the
	 * child ViewItem tree.
	 */
	const_iterator begin() const;

	/**
	 * Returns a constant depth-first iterator at the end of the child
	 * ViewItem tree.
	 */
	const_iterator end() const;

	/**
	 * Creates a list of descendant signals filtered by type.
	 */
	template<class T>
	vector< shared_ptr<T> > list_by_type() {
		vector< shared_ptr<T> > items;
		for (const auto &r : *this) {
			shared_ptr<T> p = dynamic_pointer_cast<T>(r);
			if (p)
				items.push_back(p);
		}

		return items;
	}

protected:
	item_list items_;
};

} // namespace trace
} // namespace views
} // namespace pv

#endif // PULSEVIEW_PV_VIEWS_TRACEVIEW_VIEWITEMOWNER_HPP
