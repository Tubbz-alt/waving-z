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

#include "dsp.h"

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
    union
    {
        frame_control_0_t frame_control_0;
        uint8_t fc0;
    };
    union
    {
        frame_control_1_t frame_control_1;
        uint8_t fc1;
    };
    uint8_t length;
    uint8_t dest_node_id;
    uint8_t command_class;
} __attribute__((packed));

static_assert(sizeof(packet_t) == 10, "Assumption broken");

/// CRC8 Checksum calculator
template <typename T>
typename std::iterator_traits<T>::value_type
checksum(T begin, T end)
{
    return std::accumulate(begin, end, 0xff, std::bit_xor<uint8_t>());
}

/// Convert double IQ into (unsigned) chars
template <typename Byte>
struct complex8_convert
{
    std::pair<Byte, Byte> operator()(double i, double q)
    {
        check_range(i, q);
        if (std::is_signed<Byte>::value)
        {
            return output_signed8(i, q);
        }
        else
        {
            return output_unsigned8(i, q);
        }
    }

  private:
    void check_range(double i, double q)
    {
        if (std::abs(i * A_m) > 127.0 || std::abs(q * A_m) > 127.0)
        {
            throw std::runtime_error("Value too big!");
        }
    }

    std::pair<Byte, Byte> output_unsigned8(double i, double q)
    {
        double offset = 127.0;
        return std::make_pair((Byte)(i * A_m + offset),
                              (Byte)(q * A_m + offset));
    }

    std::pair<Byte, Byte> output_signed8(double i, double q)
    {
        return std::make_pair((Byte)(i * A_m), (Byte)(q * A_m));
    }
    const double A_m = 100.0;
};

template <typename Byte, typename It, size_t SAMPLE_RATE = 2000000,
          size_t baud = 40000>
std::vector<std::pair<Byte, Byte>>
encode(It payload_begin, It payload_end)
{
    static_assert(SAMPLE_RATE % baud == 0, "Sample rate should be a multiple "
                                           "of baud rate in order to produce a "
                                           "coherent modulator.");

    constexpr size_t dfreq = 20000;

    static_assert(baud % 2*dfreq == 0, "baud rate need to be an integer multiple "
                                     "of dfreq in order to produce a coherent "
                                     "modulator.");

    constexpr uint8_t preamble = 0x55; // 10101010...10101010 frame preamble
    constexpr uint8_t SOF = 0xF0;      // Start of frame mark
    constexpr double f0_mul = 0.5;
    constexpr double f1_mul = 2.5;

    static_assert(f1_mul - f0_mul == 2.0,
                  "The Frequency shift is should be 40KHz");

    constexpr size_t Ts = SAMPLE_RATE / baud;

    std::vector<std::pair<Byte, Byte>> iq;

    double gain;
    std::array<double, 5> a1, b1;
    std::tie(gain, b1, a1) = butter_lp<4>(SAMPLE_RATE, 75000);
    iir_filter<4> lp1(gain, b1, a1);
    auto lp2 = lp1;

    complex8_convert<Byte> convert_iq;

    // .01" silence
    for (int ii(0); ii != SAMPLE_RATE / 100; ++ii) {
        iq.emplace_back(convert_iq(lp1(0.0), lp2(0.0)));
    }

    size_t tt = 0;

    // preamble
    for (size_t ii(0); ii != 160; ++ii) {
        double f_shift =
            (((preamble << (ii % 8)) & 0x80) ? f1_mul : f0_mul) * dfreq;
        for (int kk(0); kk != Ts; ++kk) {
            double i =
              lp1(sin(2.0 * M_PI * f_shift * (double)tt / SAMPLE_RATE));
            double q =
              lp2(cos(2.0 * M_PI * f_shift * (double)tt / SAMPLE_RATE));
            iq.emplace_back(convert_iq(i, q));
            ++tt;
        }
    }

    // SOF
    for (size_t ii(0); ii != 8; ++ii) {
        double f_shift = (((SOF << ii) & 0x80) ? f1_mul : f0_mul) * dfreq;
        for (int kk(0); kk != Ts; ++kk) {
            double i =
              lp1(sin(2.0 * M_PI * f_shift * (double)tt / SAMPLE_RATE));
            double q =
              lp2(cos(2.0 * M_PI * f_shift * (double)tt / SAMPLE_RATE));
            iq.emplace_back(convert_iq(i, q));
            ++tt;
        }
    }

    // payload
    for (It ch = payload_begin; ch != payload_end; ++ch) {
        for (size_t ii(0); ii != 8; ++ii) {
            double f_shift = (((*ch << ii) & 0x80) ? f1_mul : f0_mul) * dfreq;
            for (int kk(0); kk != Ts; ++kk) {
                double i =
                  lp1(sin(2.0 * M_PI * f_shift * (double)tt / SAMPLE_RATE));
                double q =
                  lp2(cos(2.0 * M_PI * f_shift * (double)tt / SAMPLE_RATE));
                iq.emplace_back(convert_iq(i, q));
                ++tt;
            }
        }
    }

    // 1" silence (it seems that this is needed by HackRF One)
    for (int ii(0); ii != (int)SAMPLE_RATE; ++ii) {
        iq.emplace_back(convert_iq(lp1(0.0), lp2(0.0)));
    }

    return iq;
}

/// Debug print a packet
template <typename It>
inline void
zwave_print(It data_begin, It data_end, size_t start_sample, size_t end_sample)
{
    size_t len = data_end - data_begin;
    std::cout << std::dec << std::setfill(' ') << std::setw(0);
    std::cout << "[" << start_sample << ", " << end_sample << "] ";
    if (len < sizeof(packet_t) ||
        checksum(data_begin, data_end - 1) != data_begin[len - 1])
    {
        std::cout << "[ ] ";
    }
    else
    {
        std::cout << "[x] ";
    }
    packet_t& p = *(packet_t*)data_begin;
    std::cout << std::hex << std::setfill('0') << std::setw(2)
              << "HomeId: " << ((uint32_t)p.home_id3 | p.home_id2 << 8 |
                                p.home_id1 << 16 | p.home_id0 << 24)
              << ", SourceNodeId: " << (int)p.source_node_id << std::hex
              << ", FC0: " << (int)p.fc0 << ", FC1: " << (int)p.fc1 << std::dec
              << ", FC[speed=" << p.frame_control_0.speed
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
        std::cout << std::setw(2) << (int)data_begin[i] << " ";
    }
    std::cout << std::endl;
}
}
