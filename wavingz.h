//
// Copyright (C) 2016 Mirko Maischberger <mirko.maischberger@gmail.com>
//
// This file is part of WavingZ.
//
// WavingZ is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// WavingZ is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with Foobar.  If not, see <http://www.gnu.org/licenses/>.
//

#pragma once

#include <chrono>
#include <iomanip>
#include <iostream>

namespace wavingz
{

/// Frame control, first byte
struct frame_control_0_t
{
    uint16_t header_type : 4;
    uint16_t speed : 1;
    uint16_t low_power : 1;
    uint16_t ack_request : 1;
    uint16_t routed : 1;
} __attribute__((packed));

static_assert(sizeof(frame_control_0_t) == 1, "Assumption broken");

/// Frame control, second byte
struct frame_control_1_t
{
    uint16_t sequence_number : 4;
    uint16_t beaming_info : 4;
} __attribute__((packed));

static_assert(sizeof(frame_control_1_t) == 1, "Assumption broken");

/// Z-Wave compatible header
///
/// We read byte by byte to be endianness agnostic; Network order is Big Endian,
/// so in order to decode the HomeID we need to
///    ((uint32_t)p.home_id3 | p.home_id2 << 8 | p.home_id1 << 16 | p.home_id0
///    << 24)
///
struct packet_t
{
    uint8_t home_id0;
    uint8_t home_id1;
    uint8_t home_id2;
    uint8_t home_id3;
    uint8_t source_node_id;
    frame_control_0_t frame_control_0;
    frame_control_1_t frame_control_1;
    uint8_t length;
    uint8_t dest_node_id;
    uint8_t command_class;
} __attribute__((packed));

static_assert(sizeof(packet_t) == 10, "Assumption broken");

/// CRC8 Checksum calculator
template<typename T>
typename std::iterator_traits<T>::value_type
checksum(T begin, T end)
{
    return std::accumulate(begin, end, 0xff, std::bit_xor<uint8_t>());
}

/// Debug print a packet
inline void
zwave_print(unsigned char* data, int len)
{
    std::chrono::milliseconds ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch());
    std::cerr << std::dec << std::setfill(' ') << std::setw(0);
    std::cerr << "[" << ms.count() << "] ";
    if (len < sizeof(packet_t) ||
        checksum(data, data + len - 1) != data[len - 1])
    {
        std::cerr << "[ ] ";
    }
    else
    {
        std::cerr << "[x] ";
    }
    packet_t& p = *(packet_t*)data;
    std::cerr << std::hex << std::setfill('0') << std::setw(2)
              << "HomeId: " << ((uint32_t)p.home_id3 | p.home_id2 << 8 | p.home_id1 << 16 | p.home_id0 << 24)
              << ", SourceNodeId: " << (int)p.source_node_id << std::hex
              << std::dec << ", FC[speed=" << p.frame_control_0.speed
              << " low_power=" << p.frame_control_0.low_power
              << " ack_request=" << p.frame_control_0.ack_request
              << " header_type=" << p.frame_control_0.header_type
              << " beaming_info=" << p.frame_control_1.beaming_info
              << " seq=" << p.frame_control_1.sequence_number
              << "], Length: " << std::dec << (int)p.length
              << ", DestNodeId: " << std::dec << (int)p.dest_node_id
              << ", CommandClass: " << std::dec << (int)p.command_class
              << ", Payload: " << std::hex << std::setfill('0');
    for (int i = sizeof(packet_t); i < len - 1; i++) {
        std::cerr << std::setw(2) << (int)data[i] << " ";
    }
    std::cerr << std::endl;
}


}
