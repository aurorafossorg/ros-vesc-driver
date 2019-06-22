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

#ifndef VESC_DRIVER_VESC_INTERFACE_H_
#define VESC_DRIVER_VESC_INTERFACE_H_

#include <string>
#include <sstream>
#include <exception>
#include <stdexcept>

#include <boost/function.hpp>
#include <boost/noncopyable.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>

#include "vesc_driver/vesc_packet.h"

namespace vesc_driver
{

/**
 * Class providing an interface to the Vedder VESC motor controller via a serial port interface.
 */
class VescInterface : private boost::noncopyable
{
public:

  typedef boost::function<void (const VescPacketConstPtr&)> PacketHandlerFunction;
  typedef boost::function<void (const std::string&)> ErrorHandlerFunction;

  /**
   * Creates a VescInterface object. Opens the serial port interface to the VESC if @p port is not
   * empty, otherwise the serial port remains closed until connect() is called.
   *
   * @param port Address of the serial port, e.g. '/dev/ttyUSB0'.
   * @param packet_handler Function this class calls when a VESC packet is received.
   * @param error_handler Function this class calls when an error is detected, such as a bad
   *                      checksum.
   *
   * @throw SerialException
   */
  VescInterface(const std::string& port = std::string(),
                const PacketHandlerFunction& packet_handler = PacketHandlerFunction(),
                const ErrorHandlerFunction& error_handler = ErrorHandlerFunction());

  /**
   * VescInterface destructor.
   */
  ~VescInterface();

  /**
   * Sets / updates the function that this class calls when a VESC packet is received.
   */
  void setPacketHandler(const PacketHandlerFunction& handler);

  /**
   * Sets / updates the function that this class calls when an error is detected, such as a bad
   * checksum.
   */
  void setErrorHandler(const ErrorHandlerFunction& handler);

  /**
   * Opens the serial port interface to the VESC.
   *
   * @throw SerialException
   */
  void connect(const std::string& port);

  /**
   * Closes the serial port interface to the VESC.
   */
  void disconnect();

  /**
   * Gets the status of the serial interface to the VESC.
   *
   * @return Returns true if the serial port is open, false otherwise.
   */
  bool isConnected() const;

  /**
   * Send a VESC packet.
   */
  void send(const VescPacket& packet);

  void requestFWVersion();
  void requestState();
  void setDutyCycle(double duty_cycle);
  void setCurrent(double current);
  void setBrake(double brake);
  void setSpeed(double speed);
  void setPosition(double position);
  void setServo(double servo);

private:
  // Pimpl - hide serial port members from class users
  class Impl;
  boost::scoped_ptr<Impl> impl_;
};

// todo: review
class SerialException : public std::exception
{
  // Disable copy constructors
  SerialException& operator=(const SerialException&);
  std::string e_what_;
public:
  SerialException (const char *description) {
      std::stringstream ss;
      ss << "SerialException " << description << " failed.";
      e_what_ = ss.str();
  }
  SerialException (const SerialException& other) : e_what_(other.e_what_) {}
  virtual ~SerialException() throw() {}
  virtual const char* what () const throw () {
    return e_what_.c_str();
  }
};

} // namespace vesc_driver

#endif // VESC_DRIVER_VESC_INTERFACE_H_
