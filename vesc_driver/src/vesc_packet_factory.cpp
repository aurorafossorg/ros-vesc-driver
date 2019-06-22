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

#include "vesc_driver/vesc_packet_factory.h"

#include <cassert>
#include <iterator>

#include <boost/range/begin.hpp>
#include <boost/range/distance.hpp>
#include <boost/range/end.hpp>

#include "vesc_driver/vesc_packet.h"

namespace vesc_driver
{

/** Construct map on first use */
VescPacketFactory::FactoryMap* VescPacketFactory::getMap()
{
  static FactoryMap m;
  return &m;
}

void VescPacketFactory::registerPacketType(int payload_id, CreateFn fn)
{
  FactoryMap* p_map(getMap());
  assert(0 == p_map->count(payload_id));
  (*p_map)[payload_id] = fn;
}

/** Helper function for when createPacket can not create a packet */
VescPacketPtr createFailed(int* p_num_bytes_needed, std::string* p_what,
                           const std::string& what, int num_bytes_needed = 0)
{
  if (p_num_bytes_needed != NULL) *p_num_bytes_needed = num_bytes_needed;
  if (p_what != NULL) *p_what = what;
  return VescPacketPtr();
}

VescPacketPtr VescPacketFactory::createPacket(const Buffer::const_iterator& begin,
                                              const Buffer::const_iterator& end,
                                              int* num_bytes_needed, std::string* what)
{
  // initialize output variables
  if (num_bytes_needed != NULL) *num_bytes_needed = 0;
  if (what != NULL) what->clear();

  // need at least VESC_MIN_FRAME_SIZE bytes in buffer
  int buffer_size(std::distance(begin, end));
  if (buffer_size < VescFrame::VESC_MIN_FRAME_SIZE)
    return createFailed(num_bytes_needed, what, "Buffer does not contain a complete frame",
                        VescFrame::VESC_MIN_FRAME_SIZE - buffer_size);

  // buffer must begin with a start-of-frame
  if (VescFrame::VESC_SOF_VAL_SMALL_FRAME != *begin &&
      VescFrame::VESC_SOF_VAL_LARGE_FRAME != *begin)
    return createFailed(num_bytes_needed, what, "Buffer must begin with start-of-frame character");

  // get a view of the payload
  BufferRangeConst view_payload;
  if (VescFrame::VESC_SOF_VAL_SMALL_FRAME == *begin) {
    // payload size field is one byte
    view_payload.first = begin + 2;
    view_payload.second = view_payload.first + *(begin + 1);
  }
  else {
    assert(VescFrame::VESC_SOF_VAL_LARGE_FRAME == *begin);
    // payload size field is two bytes
    view_payload.first = begin + 3;
    view_payload.second = view_payload.first + (*(begin + 1) << 8) + *(begin + 2);
  }

  // check length
  if (boost::distance(view_payload) > VescFrame::VESC_MAX_PAYLOAD_SIZE)
    return createFailed(num_bytes_needed, what, "Invalid payload length");

  // get iterators to crc field, end-of-frame field, and a view of the whole frame
  Buffer::const_iterator iter_crc(view_payload.second);
  Buffer::const_iterator iter_eof(iter_crc + 2);
  BufferRangeConst view_frame(begin, iter_eof + 1);

  // do we have enough data in the buffer to complete the frame?
  int frame_size = boost::distance(view_frame);
  if (buffer_size < frame_size)
    return createFailed(num_bytes_needed, what, "Buffer does not contain a complete frame",
                        frame_size - buffer_size);

  // is the end-of-frame character valid?
  if (VescFrame::VESC_EOF_VAL != *iter_eof)
    return createFailed(num_bytes_needed, what, "Invalid end-of-frame character");

  // is the crc valid?
  unsigned short crc = (static_cast<unsigned short>(*iter_crc) << 8) + *(iter_crc + 1);
  VescFrame::CRC crc_calc;
  crc_calc.process_bytes(&(*view_payload.first), boost::distance(view_payload));
  if (crc != crc_calc.checksum())
    return createFailed(num_bytes_needed, what, "Invalid checksum");

  // frame looks good, construct the raw frame
  boost::shared_ptr<VescFrame> raw_frame(new VescFrame(view_frame, view_payload));

  // if the packet has a payload, construct the corresponding subclass
  if (boost::distance(view_payload) > 0) {

    // get constructor function from payload id
    FactoryMap* p_map(getMap());
    FactoryMap::const_iterator search(p_map->find(*view_payload.first));
    if (search != p_map->end()) {
      return search->second(raw_frame);
    }
    else {
      // no subclass constructor for this packet
      return createFailed(num_bytes_needed, what, "Unkown payload type.");
    }

  }
  else {
    // no payload
    return createFailed(num_bytes_needed, what, "Frame does not have a payload");
  }
}


} // namesapce vesc_driver
