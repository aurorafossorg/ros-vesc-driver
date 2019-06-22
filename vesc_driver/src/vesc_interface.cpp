/*****************************************************************************
**                                     __
**                                    / _|
**   __ _ _   _ _ __ ___  _ __ __ _  | |_ ___  ___ ___
**  / _` | | | | '__/ _ \| '__/ _` | |  _/ _ \/ __/ __|
** | (_| | |_| | | | (_) | | | (_| | | || (_) \__ \__ \
**  \__,_|\__,_|_|  \___/|_|  \__,_| |_| \___/|___/___/
**
** Copyright (C) 2015-2016 Michael T. Boulet <boulet@ll.mit.edu>
** Copyright (C) 2012-2014 Benjamin Vedder <benjamin@vedder.se>
** Copyright (C) 2019 Aurora Free Open Source Software.
** Copyright (C) 2019 Luís Ferreira <luis@aurorafoss.org>
**
** This file is part of the Aurora Free Open Source Software. This
** organization promote free and open source software that you can
** redistribute and/or modify under the terms of the GNU Lesser General
** Public License Version 3 as published by the Free Software Foundation or
** (at your option) any later version approved by the Aurora Free Open Source
** Software Organization. The license is available in the package root path
** as 'LICENSE' file. Please review the following information to ensure the
** GNU Lesser General Public License version 3 requirements will be met:
** https://www.gnu.org/licenses/lgpl.html .
**
** Alternatively, this file may be used under the terms of the GNU General
** Public License version 3 or later as published by the Free Software
** Foundation. Please review the following information to ensure the GNU
** General Public License requirements will be met:
** http://www.gnu.org/licenses/gpl-3.0.html.
**
** NOTE: All products, services or anything associated to trademarks and
** service marks used or referenced on this file are the property of their
** respective companies/owners or its subsidiaries. Other names and brands
** may be claimed as the property of others.
**
** For more info about intellectual property visit: aurorafoss.org or
** directly send an email to: contact (at) aurorafoss.org .
**
** This project was originally licensed under BSD.
*****************************************************************************/

#include "vesc_driver/vesc_interface.h"

#include <pthread.h>

#include <string>
#include <sstream>
#include <iostream>
#include <iomanip>

#include <serial/serial.h>
#include <boost/crc.hpp>

#include "vesc_driver/vesc_packet_factory.h"

namespace vesc_driver
{

class VescInterface::Impl
{
public:
	Impl() :
		serial_(std::string(), 115200, serial::Timeout::simpleTimeout(100),
						serial::eightbits, serial::parity_none, serial::stopbits_one, serial::flowcontrol_none)
	{}

	void* rxThread(void);

	static void* rxThreadHelper(void *context)
	{
		return ((VescInterface::Impl*)context)->rxThread();
	}

	pthread_t rx_thread_;
	bool rx_thread_run_;
	PacketHandlerFunction packet_handler_;
	ErrorHandlerFunction error_handler_;
	serial::Serial serial_;
	VescFrame::CRC send_crc_;
};

void* VescInterface::Impl::rxThread(void)
{
	Buffer buffer;
	buffer.reserve(4096);

	while (rx_thread_run_) {

		int bytes_needed = VescFrame::VESC_MIN_FRAME_SIZE;
		if (!buffer.empty()) {

			// search buffer for valid packet(s)
			Buffer::iterator iter(buffer.begin());
			Buffer::iterator iter_begin(buffer.begin());
			while (iter != buffer.end()) {

				// check if valid start-of-frame character
				if (VescFrame::VESC_SOF_VAL_SMALL_FRAME == *iter ||
						VescFrame::VESC_SOF_VAL_LARGE_FRAME == *iter) {

					// good start, now attempt to create packet
					std::string error;
					VescPacketConstPtr packet =
						VescPacketFactory::createPacket(iter, buffer.end(), &bytes_needed, &error);
					if (packet) {
						// good packet, check if we skipped any data
						if (std::distance(iter_begin, iter) > 0) {
							std::ostringstream ss;
							ss << "Out-of-sync with VESC, unknown data leading valid frame. Discarding "
								 << std::distance(iter_begin, iter) << " bytes.";
							error_handler_(ss.str());
						}
						// call packet handler
						packet_handler_(packet);
						// update state
						iter = iter + packet->frame().size();
						iter_begin = iter;
						// continue to look for another frame in buffer
						continue;
					}
					else if (bytes_needed > 0) {
						// need more data, break out of while loop
						break; // for (iter_sof...
					}
					else {
						// else, this was not a packet, move on to next byte
						error_handler_(error);
					}
				}

				iter++;
			}

			// if iter is at the end of the buffer, more bytes are needed
			if (iter == buffer.end())
				bytes_needed = VescFrame::VESC_MIN_FRAME_SIZE;

			// erase "used" buffer
			if (std::distance(iter_begin, iter) > 0) {
				std::ostringstream ss;
				ss << "Out-of-sync with VESC, discarding " << std::distance(iter_begin, iter) << " bytes.";
				error_handler_(ss.str());
			}
			buffer.erase(buffer.begin(), iter);
		}

		// attempt to read at least bytes_needed bytes from the serial port
		int bytes_to_read =
			std::max(bytes_needed, std::min(4096, static_cast<int>(serial_.available())));
		int bytes_read = serial_.read(buffer, bytes_to_read);
		if (bytes_needed > 0 && 0 == bytes_read && !buffer.empty()) {
			error_handler_("Possibly out-of-sync with VESC, read timout in the middle of a frame.");
		}

	}
}


VescInterface::VescInterface(const std::string& port,
														 const PacketHandlerFunction& packet_handler,
														 const ErrorHandlerFunction& error_handler) :
	impl_(new Impl())
{
	setPacketHandler(packet_handler);
	setErrorHandler(error_handler);
	// attempt to conect if the port is specified
	if (!port.empty())
		connect(port);
}

VescInterface::~VescInterface()
{
	disconnect();
}

void VescInterface::setPacketHandler(const PacketHandlerFunction& handler)
{
	// todo - definately need mutex
	impl_->packet_handler_ = handler;
}

void VescInterface::setErrorHandler(const ErrorHandlerFunction& handler)
{
	// todo - definately need mutex
	impl_->error_handler_ = handler;
}

void VescInterface::connect(const std::string& port)
{
	// todo - mutex?

	if (isConnected()) {
		throw SerialException("Already connected to serial port.");
	}

	// connect to serial port
	try {
		impl_->serial_.setPort(port);
		impl_->serial_.open();
	}
	catch (const std::exception& e) {
			std::stringstream ss;
			ss << "Failed to open the serial port to the VESC. " << e.what();
			throw SerialException(ss.str().c_str());
	}

	// start up a monitoring thread
	impl_->rx_thread_run_ = true;
	int result =
		pthread_create(&impl_->rx_thread_, NULL, &VescInterface::Impl::rxThreadHelper, impl_.get());
	assert(0 == result);
}

void VescInterface::disconnect()
{
	// todo - mutex?

	if (isConnected()) {
		// bring down read thread
		impl_->rx_thread_run_ = false;
		int result = pthread_join(impl_->rx_thread_, NULL);
		assert(0 == result);

		impl_->serial_.close();
	}
}

bool VescInterface::isConnected() const
{
	return impl_->serial_.isOpen();
}

void VescInterface::send(const VescPacket& packet)
{
	size_t written = impl_->serial_.write(packet.frame());
	if (written != packet.frame().size()) {
		std::stringstream ss;
		ss << "Wrote " << written << " bytes, expected " << packet.frame().size() << ".";
		throw SerialException(ss.str().c_str());
	}
}

void VescInterface::requestFWVersion()
{
	send(VescPacketRequestFWVersion());
}

void VescInterface::requestState()
{
	send(VescPacketRequestValues());
}

void VescInterface::setDutyCycle(double duty_cycle)
{
	send(VescPacketSetDuty(duty_cycle));
}

void VescInterface::setCurrent(double current)
{
	send(VescPacketSetCurrent(current));
}

void VescInterface::setBrake(double brake)
{
	send(VescPacketSetCurrentBrake(brake));
}

void VescInterface::setSpeed(double speed)
{
	send(VescPacketSetRPM(speed));
}

void VescInterface::setPosition(double position)
{
	send(VescPacketSetPos(position));
}

void VescInterface::setServo(double servo)
{
	send(VescPacketSetServoPos(servo));
}

} // namespace vesc_driver
