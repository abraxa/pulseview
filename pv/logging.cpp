/*
 * This file is part of the PulseView project.
 *
 * Copyright (C) 2018 Soeren Apel <soeren@apelpie.net>
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

#include <iostream>

#include <QApplication>

#ifdef ENABLE_DECODE
#include <libsigrokdecode/libsigrokdecode.h> /* First, so we avoid a _POSIX_C_SOURCE warning. */
#endif

#include <libsigrokcxx/libsigrokcxx.hpp>

#include "pv/logging.hpp"
#include "pv/globalsettings.hpp"

using std::cout;
using std::endl;
using std::lock_guard;

namespace pv {

Logging logging;

const int Logging::MIN_BUFFER_SIZE = 10;
const int Logging::MAX_BUFFER_SIZE = 50000;

static sr_log_callback prev_sr_log_cb;
static void *prev_sr_log_cb_data;

#ifdef ENABLE_DECODE
static srd_log_callback prev_srd_log_cb;
static void *prev_srd_log_cb_data;
#endif

Logging::~Logging()
{
	qInstallMessageHandler(nullptr);
	if (prev_sr_log_cb)
		sr_log_callback_set(prev_sr_log_cb, prev_sr_log_cb_data);
	prev_sr_log_cb = nullptr;
	prev_sr_log_cb_data = nullptr;
#ifdef ENABLE_DECODE
	if (prev_srd_log_cb)
		srd_log_callback_set(prev_srd_log_cb, prev_srd_log_cb_data);
	prev_srd_log_cb = nullptr;
	prev_srd_log_cb_data = nullptr;
#endif

	GlobalSettings::remove_change_handler(this);
}

void Logging::init()
{
	GlobalSettings settings;

	buffer_size_ =
		settings.value(GlobalSettings::Key_Log_BufferSize).toInt();

	buffer_.reserve(buffer_size_);

	qInstallMessageHandler(log_pv);
	sr_log_callback_get(&prev_sr_log_cb, &prev_sr_log_cb_data);
	sr_log_callback_set(log_sr, nullptr);
#ifdef ENABLE_DECODE
	srd_log_callback_get(&prev_srd_log_cb, &prev_srd_log_cb_data);
	srd_log_callback_set(log_srd, nullptr);
#endif

	GlobalSettings::add_change_handler(this);
}

int Logging::get_log_level() const
{
	// We assume that libsigrok and libsrd always have the same log level
	return sr_log_loglevel_get();
}

void Logging::set_log_level(int level)
{
	sr_log_loglevel_set(level);
#ifdef ENABLE_DECODE
	srd_log_loglevel_set(level);
#endif
}

QString Logging::get_log() const
{
	return buffer_.join("<br />\n");
}

void Logging::log(const QString &text, int source)
{
	lock_guard<mutex> log_lock(log_mutex_);

	if (buffer_.size() >= buffer_size_)
		buffer_.removeFirst();

	QString s;

	if (text.contains("warning", Qt::CaseInsensitive)) {
		s = QString("<font color=\"darkorange\">%1</font>").arg(text);
		goto out;
	}

	if (text.contains("error", Qt::CaseInsensitive)) {
		s = QString("<font color=\"darkred\">%1</font>").arg(text);
		goto out;
	}

	switch (source) {
	case LogSource_pv:
		s = QString("<font color=\"darkMagenta\">pv: %1</font>").arg(text);
		break;
	case LogSource_sr:
		s = QString("<font color=\"darkGreen\">sr: %1</font>").arg(text);
		break;
	case LogSource_srd:
		s = QString("<font color=\"olive\">srd: %1</font>").arg(text);
		break;
	default:
		s = text;
		break;
	}

out:
	buffer_.append(s);

	// If we're tearing down the program, sending out notifications to UI
	// elements that can no longer function properly is a bad idea
	if (!QApplication::closingDown())
		logged_text(s);
}

void Logging::log_pv(QtMsgType type, const QMessageLogContext &context, const QString &msg)
{
	(void)type;
	(void)context;

	logging.log(msg, LogSource_pv);

	cout << msg.toUtf8().data() << endl;
}

int Logging::log_sr(void *cb_data, int loglevel, const char *format, va_list args)
{
	va_list args2;

	(void)cb_data;

	va_copy(args2, args);
	if (prev_sr_log_cb)
		prev_sr_log_cb(prev_sr_log_cb_data, loglevel, format, args2);
	va_end(args2);

	char *text = g_strdup_vprintf(format, args);
	logging.log(QString::fromUtf8(text), LogSource_sr);
	g_free(text);

	return SR_OK;
}

#ifdef ENABLE_DECODE
int Logging::log_srd(void *cb_data, int loglevel, const char *format, va_list args)
{
	va_list args2;

	(void)cb_data;

	va_copy(args2, args);
	if (prev_srd_log_cb)
		prev_srd_log_cb(prev_srd_log_cb_data, loglevel, format, args2);
	va_end(args2);

	char *text = g_strdup_vprintf(format, args);

	QString s = QString::fromUtf8(text);
	for (QString& substring : s.split("\n", QString::SkipEmptyParts))
			logging.log(substring, LogSource_srd);
	g_free(text);

	return SR_OK;
}
#endif

void Logging::on_setting_changed(const QString &key, const QVariant &value)
{
	if (key == GlobalSettings::Key_Log_BufferSize) {
		// Truncate buffer if needed
		const int delta = buffer_.size() - value.toInt();
		if (delta > 0)
			buffer_.erase(buffer_.begin(), buffer_.begin() + delta);

		buffer_size_ = value.toInt();
		buffer_.reserve(buffer_size_);
	}
}

} // namespace pv
