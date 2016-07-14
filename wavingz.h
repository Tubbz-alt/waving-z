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

#include <boost/optional.hpp>

#include <bitset>
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

/// Encode the payload into an IQ signal (cu8 or cs8 depending on Byte type)
template <typename Byte, typename It, size_t SAMPLE_RATE = 2000000,
          size_t baud = 40000>
std::vector<std::pair<Byte, Byte>>
encode(It payload_begin, It payload_end)
{
    constexpr size_t dfreq = 20000;
    constexpr uint8_t preamble = 0x55; // 10101010...10101010 frame preamble
    constexpr uint8_t SOF = 0xF0;      // Start of frame mark
    constexpr double f0_mul = .5;
    constexpr double f1_mul = 2.5;
    static_assert(SAMPLE_RATE % baud == 0,
                  "Sample rate needs to be a multiple of baud rate");
    static_assert(dfreq * (int)(f1_mul - f0_mul) % baud == 0,
                  "Phase coherency rule not satisfied");
    static_assert(f1_mul - f0_mul == 2.0,
                  "The Frequency shift should be 40KHz");
    constexpr size_t Ts = SAMPLE_RATE / baud;

    std::vector<std::pair<Byte, Byte>> iq;

    double gain;
    std::array<double, 4> a1, b1;
    std::tie(gain, b1, a1) = butter_lp<3>(SAMPLE_RATE, 60000*2.5);
    iir_filter<3> lp1(gain, b1, a1);
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
zwave_print(It data_begin, It data_end)
{
    size_t len = data_end - data_begin;
    packet_t& p = *(packet_t*)data_begin;
    if (len < sizeof(packet_t) || len < p.length+1 ||
        checksum(data_begin, data_begin + (size_t)p.length) != data_begin[p.length])
    {
        std::cout << "[ ] ";
    }
    else
    {
        std::cout << "[x] ";
    }
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
              << ", CommandClass: " << std::hex << (int)p.command_class
              << ", Payload: " << std::hex << std::setfill('0');
    for (int i = sizeof(packet_t); i < p.length; i++) {
        std::cout << std::setw(2) << (int)data_begin[i] << " ";
    }
    std::cout << std::endl;
}

// demodulation state machine
namespace demod_sm
{

struct symbol_sm_t;

// -----------------------------------------------------------------------------

namespace symbol_sm
{

struct state_base_t
{
    virtual void process(symbol_sm_t& ctx, const boost::optional<bool>& symbol) = 0;
};

// Detecting the first nibble of the SOF (0xF)
struct start_of_frame_1_t : public state_base_t
{
    void process(symbol_sm_t& ctx, const boost::optional<bool>& symbol) override;
private:
    size_t cnt = 0;
};

// Parsing the second nibble of the SOF (0x0)
struct start_of_frame_0_t : public state_base_t
{
    void process(symbol_sm_t& ctx, const boost::optional<bool>& symbol) override;
private:
    size_t cnt = 0;
};

// Pushing data into payload
struct payload_t : public state_base_t
{
    void process(symbol_sm_t& ctx, const boost::optional<bool>& symbol) override;
private:
    std::vector<uint8_t> payload;
    std::bitset<8> b = 0;
    size_t cnt = 0;
};

} // namespace

// -----------------------------------------------------------------------------

struct symbol_sm_t
{
    symbol_sm_t(const std::function<void(uint8_t*, uint8_t*)>& callback)
      : callback(callback)
      , current_state_m(new symbol_sm::start_of_frame_1_t())
    {
    }
    // sample can be 0, 1 or none (no signal)
    void process(const boost::optional<bool>& symbol);
    void state(std::unique_ptr<symbol_sm::state_base_t>&& next_state);
    std::function<void(uint8_t*, uint8_t*)> callback;
private:
    std::unique_ptr<symbol_sm::state_base_t> current_state_m;
};

struct sample_sm_t;

// -----------------------------------------------------------------------------

namespace sample_sm
{

struct state_base_t
{
    virtual void process(sample_sm_t& ctx, const boost::optional<bool>& sample) = 0;
};

struct idle_t : public state_base_t
{
    void process(sample_sm_t& ctx, const boost::optional<bool>& sample) override;
};

struct lead_in_t : public state_base_t
{
    lead_in_t()
      : counter(0)
      , last_sample(0)
    {
    }
    void process(sample_sm_t& ctx, const boost::optional<bool>& sample) override;
private:
    size_t counter;
    bool last_sample;
};

struct preamble_t : public state_base_t
{
    preamble_t(bool last_sample)
        : last_sample(last_sample)
    {}
    void process(sample_sm_t& ctx, const boost::optional<bool>& sample) override;
    size_t symbols_counter = 0;
    size_t samples_counter = 0;
    bool last_sample;
};

struct bitlock_t : public state_base_t
{
    bitlock_t(double samples_per_symbol)
      : samples_per_symbol(samples_per_symbol)
      , num_high(0)
      , num_samples(0.0)
    {}
    void process(sample_sm_t& ctx, const boost::optional<bool>& sample) override;
    const double samples_per_symbol;
    size_t num_high;
    double num_samples;
};

} // namespace

// -----------------------------------------------------------------------------

struct sample_sm_t
{
    sample_sm_t(size_t sample_rate, symbol_sm_t& sym_sm);
    // sample can be 0, 1 or none (no signal)
    void process(const boost::optional<bool>& sample);
    void state(std::unique_ptr<sample_sm::state_base_t>&& next_state);
    bool preamble() { return
            typeid(*current_state_m) == typeid(sample_sm::preamble_t) ||
            typeid(*current_state_m) == typeid(sample_sm::lead_in_t);
    }
    bool idle() { return typeid(*current_state_m) == typeid(sample_sm::idle_t); }
    void emit(const boost::optional<bool>& symbol);
    const size_t sample_rate;
private:
    std::reference_wrapper<symbol_sm_t> sym_sm;
    std::unique_ptr<sample_sm::state_base_t> current_state_m;
};

} // namespace
} // namespace
