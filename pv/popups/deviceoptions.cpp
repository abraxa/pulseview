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

#include <QFormLayout>
#include <QListWidget>

#include <libsigrokcxx/libsigrokcxx.hpp>

#include "deviceoptions.hpp"  /* With the full path, lupdate can't find the header */

#include "pv/prop/property.hpp"

using std::shared_ptr;

namespace pv {
namespace popups {

DeviceOptions::DeviceOptions(shared_ptr<sigrok::Device> device, QWidget *parent) :
	Popup(parent),
	device_(device),
	layout_(this),
	binding_(device)
{
	setLayout(&layout_);

	layout_.addWidget(binding_.get_property_form(this, true));
}

pv::binding::Device& DeviceOptions::binding()
{
	return binding_;
}

void DeviceOptions::show()
{
	// Update device config widgets with the current values supplied by the
	// driver before actually showing the popup dialog
	binding_.update_property_widgets();

	Popup::show();
}

} // namespace popups
} // namespace pv
