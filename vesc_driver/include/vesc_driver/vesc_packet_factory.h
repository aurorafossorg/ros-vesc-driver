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

#ifndef VESC_DRIVER_VESC_PACKET_FACTORY_H_
#define VESC_DRIVER_VESC_PACKET_FACTORY_H_

#include <vector>
#include <map>
#include <string>

#include <boost/noncopyable.hpp>
#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>

#include "vesc_driver/v8stdint.h"
#include "vesc_driver/vesc_packet.h"

namespace vesc_driver
{

/**
 * Class for creating VESC packets from raw data.
 */
class VescPacketFactory : private boost::noncopyable
{
public:
  /** Return the global factory object */
  static VescPacketFactory* getFactory();

  /**
   * Create a VescPacket from a buffer (factory function). Packet must start (start of frame
   * character) at @p begin and complete (end of frame character) before *p end. The buffer element
   * at @p end is not examined, i.e. it can be the past-the-end element. Only returns a packet if
   * the packet is valid, i.e. valid size, matching checksum, complete etc. An empty pointer is
   * returned if a packet cannot be found or if it is invalid. If a valid packet is not found,
   * optional output parameter @what is set to a string providing a reason why a packet was not
   * found. If a packet was not found because additional bytes are needed on the buffer, optional
   * output parameter @p num_bytes_needed will contain the number of bytes needed to either
   * determine the size of the packet or complete the packet. Output parameters @p num_bytes_needed
   * and @p what will be set to 0 and empty if a valid packet is found.
   *
   * @param begin[in] Iterator to a buffer at the start-of-frame character
   * @param end[in] Iterator to the buffer past-the-end element.
   * @param num_bytes_needed[out] Number of bytes needed to determine the packet size or complete
   *                              the frame.
   * @param what[out] Human readable string giving a reason why the packet was not found.
   *
   * @return Pointer to a valid VescPacket if successful. Otherwise, an empty pointer.
   */
  static VescPacketPtr createPacket(const Buffer::const_iterator& begin,
                                    const Buffer::const_iterator& end,
                                    int* num_bytes_needed, std::string* what);

  typedef boost::function<VescPacketPtr(boost::shared_ptr<VescFrame>)> CreateFn;

  /** Register a packet type with the factory. */
  static void registerPacketType(int payload_id, CreateFn fn);

private:

  typedef std::map<int, CreateFn > FactoryMap;
  static FactoryMap* getMap();
};

/** Use this macro to register packets */
#define REGISTER_PACKET_TYPE(id, klass)   \
class klass##Factory \
{ \
public: \
  klass##Factory() \
  { \
    VescPacketFactory::registerPacketType((id), &klass##Factory::create); \
  } \
  static VescPacketPtr create(boost::shared_ptr<VescFrame> frame) \
  { \
    return VescPacketPtr(new klass(frame)); \
  } \
}; \
static klass##Factory global_##klass##Factory;

} // namespace vesc_driver

#endif // VESC_DRIVER_VESC_PACKET_FACTORY_H_
