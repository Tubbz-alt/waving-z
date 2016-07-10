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

#include "dsp.h"
#include "wavingz.h"

#include <cstdio>
#include <cstdint>
#include <complex>
#include <unistd.h>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <cassert>
#include <functional>
#include <algorithm>

#include <boost/circular_buffer.hpp>

using namespace std;

// assumptions were made (e.g. on endianness)
struct frame_control_t
{
    uint16_t header_type : 4;
    uint16_t speed : 1;
    uint16_t low_power : 1;
    uint16_t ack_request : 1;
    uint16_t routed : 1;
    uint16_t sequence_number : 4;
    uint16_t beaming_info : 4;
} __attribute__((packed));

static_assert(sizeof(frame_control_t) == 2, "Assumption broken");

struct packet_t
{
    uint32_t home_id;
    uint8_t source_node_id;
    frame_control_t frame_control;
    uint8_t length;
    uint8_t dest_node_id;
    uint8_t command_class;
} __attribute__((packed));

static_assert(sizeof(packet_t) == 10, "Assumption broken");


/// Print z-wave packet
void
zwave_print(unsigned char* data, int len)
{
    chrono::milliseconds ms = chrono::duration_cast<chrono::milliseconds>(
      chrono::system_clock::now().time_since_epoch());
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
    std::cerr << std::hex << setfill('0') << std::setw(2)
              << "HomeId: " << p.home_id
              << ", SourceNodeId: " << (int)p.source_node_id << std::hex
              << ", FC: " << *reinterpret_cast<uint16_t*>(&p.frame_control)
              << std::dec << ", FC[speed=" << p.frame_control.speed
              << " low_power=" << p.frame_control.low_power
              << " ack_request=" << p.frame_control.ack_request
              << " header_type=" << p.frame_control.header_type
              << " beaming_info=" << p.frame_control.beaming_info
              << " seq=" << p.frame_control.sequence_number
              << "], Length: " << std::dec << (int)p.length
              << ", DestNodeId: " << std::dec << (int)p.dest_node_id
              << ", CommandClass: " << std::dec << (int)p.command_class
              << ", Payload: " << std::hex << setfill('0');
    for (int i = sizeof(packet_t); i < len - 1; i++) {
        std::cerr << std::setw(2) << (int)data[i] << " ";
    }
    std::cerr << std::endl;
}

/// Simple arctan demodulator
struct atan_fm_demodulator
{
    atan_fm_demodulator()
      : s1(0)
    {
    }
    /// Q&I
    double operator()(double re, double im)
    {
        std::complex<double> s(re, im);
        double d = std::arg(std::conj(s1) * s);
        s1 = s;
        return d;
    }
    std::complex<double> s1;
};

struct frame_state
{
    unsigned int bit_count;
    unsigned int data_len;
    unsigned char data[64];

    bool last_bit;

    int b_cnt;

    enum
    {
        B_PREAMP,
        B_SOF0,
        B_SOF1,
        B_DATA
    } state_b;
} fs;

enum
{
    S_IDLE,
    S_PREAMP,
    S_BITLOCK
} state = S_IDLE;

int
main(int argc, char** argv)
{
    const int SAMPLE_RATE = 2048000;
    int pre_len = 0; //  # Length of preamble bit
    int pre_cnt = 0;
    double bit_len = 0;
    double bit_cnt = 0.0;
    double wc = 0; //  # center frequency
    bool last_logic = false;
    bool hasSignal = false;
    bool msc; // Manchester
    const int lead_in = 10;
    double dr; // Datarate

    double f, s, lock;
    size_t s_num = 0; // Sample number
    size_t f_num = 0; // Z-wave frame number
    int rec_ptr = 0;
    int frame_start;

    atan_fm_demodulator demod;

    std::array<double, 5> a1, b1;
    std::tie(b1, a1) = butter_lp<4>(SAMPLE_RATE, 75000);

    iir_filter<4> lp1(b1, a1);
    auto lp2 = lp1;

    std::array<double, 4> a2, b2;
    std::tie(b2, a2) = butter_lp<3>(SAMPLE_RATE, 102400);

    iir_filter<3> freq_filter(b2, a2);

    std::array<double, 4> a3, b3;
    std::tie(b3, a3) = butter_lp<3>(SAMPLE_RATE, 10240);

    iir_filter<3> lock_filter(b3, a3);

    std::vector<int> bits;

    while (!feof(stdin)) {
        unsigned char g[1024];
        if (fread(g, 1024, 1, stdin) != 1)
        {
            continue;
        };

        for (int i = 0; i < 1024; i += 2) {
            double re = double(g[i])/255.0 - 0.5;
            double im = double(g[i + 1])/255.0 - 0.5;

            s_num++;

            double re2 = lp1(re);
            double im2 = lp2(im);
            f = demod(re2, im2);

            s = freq_filter(f);

            /*
             * lowpass filter to lock on to a preable. When this value is
             * "stable", a preamble could be present, further more the value of
             * lock, will correspond to the center frequency of the fsk (wc)
             */
            lock = lock_filter(f);

            /* If we are in bitlock mode, make sure that the signal does not
             * derivate by more than 1/2 seperation, TODO calculate 1/2
             * seperation */
            hasSignal = fabs(lock) > 0.005;

            if (hasSignal)
            {
                bool logic = (s - wc) < 0;

                if (state == S_IDLE)
                {
                    for(auto bit: bits)
                    {
                        std::cerr <<  bit << " ";
                    }
                    if (bits.size())
                        std::cerr << endl;
                    bits.clear();
                    state = S_PREAMP;
                    pre_cnt = 0;
                    pre_len = 0;
                    frame_start = rec_ptr;
                    wc = lock;
                }
                else if (state == S_PREAMP)
                {
                    wc = 0.95 * wc + lock * 0.05;
                    pre_len++;
                    if (logic ^ last_logic) //#edge trigger (rising and falling)
                    {
                        pre_cnt++;

                        if (pre_cnt == lead_in) //# skip the first lead_in
                        {
                            pre_len = 0;
                        }
                        else if (pre_cnt > lead_in + 20) // Minimum preamble
                                                         // length is 10 bytes
                                                         // i.e 80 bits
                        {
                            state = S_BITLOCK;
                            fs.state_b = fs.B_PREAMP;
                            fs.last_bit = not logic;
                            bits.push_back(fs.last_bit);
                            bit_len = double(pre_len) / (pre_cnt - lead_in - 1);
                            bit_cnt = 3 * bit_len / 4.0;
                            dr = SAMPLE_RATE / bit_len;
                            msc = dr < 15e3; // Should we use manchester
                                             // encoding
                            if( dr > 42e3 )
                            {
                                bits.clear();
                                state = S_IDLE;
                            }
                            else
                            {
                                std::cout << "Manchester: " << msc << std::endl;
                                std::cout << "Datarate: " << dr << std::endl;
                            }
                        }
                    }
                }
                else if (state == S_BITLOCK)
                {
                    if (logic ^ last_logic)
                    {
                        if (msc && (bit_cnt < bit_len / 2.0))
                        {
                            bit_cnt = 1 * bit_len / 4.0; //#Re-sync on edges
                        }
                        else
                        {
                            bit_cnt = 3 * bit_len / 4.0; //#Re-sync on edges
                        }
                    }
                    else
                    {
                        bit_cnt = bit_cnt + 1.0;
                    }
                    if (bit_cnt >= bit_len) // # new bit
                    {
                        bits.push_back(logic);
                        // Sub state machine
                        if (fs.state_b == fs.B_PREAMP)
                        {
                            if (logic and fs.last_bit)
                            {
                                fs.state_b = fs.B_SOF1;
                                fs.b_cnt = 1; // This was the first SOF bit
                            }
                        }
                        else if (fs.state_b == fs.B_SOF0)
                        {
                            if (not logic)
                            {
                                if (fs.b_cnt == 4)
                                {
                                    fs.b_cnt = 0;
                                    fs.data_len = 0;
                                    fs.state_b = fs.B_DATA;
                                    std::cerr << std::endl << "DATA" << std::endl;
                                }
                            }
                            else
                            {
                                // SOF0 error (bit_len);
                                std::cerr << "SOF0 Error" << std::endl;
                                state = S_IDLE;
                            }
                        }
                        else if (fs.state_b == fs.B_SOF1)
                        {
                            if (logic)
                            {
                                if (fs.b_cnt == 4)
                                {
                                    fs.b_cnt = 0;
                                    fs.state_b = fs.B_SOF0;
                                }
                            }
                            else
                            {
                                // SOF1 error
                                std::cerr << "SOF1 Error" << std::endl;
                                state = S_IDLE;
                            }
                        }
                        else if (fs.state_b == fs.B_DATA) // Payload bit
                        {
                            fs.data[fs.data_len] =
                              (fs.data[fs.data_len] << 1) | logic;
                            if ((fs.b_cnt & 7) == 0)
                            {
                                fs.data[++fs.data_len] = 0;
                            }
                        }
                        fs.last_bit = logic;
                        fs.b_cnt++;
                        bit_cnt = bit_cnt - bit_len;
                    }
                }
                last_logic = logic;
            }
            else //# No LOCKs
            {
                if (state == S_BITLOCK && fs.state_b == fs.B_DATA)
                {
                    f_num++;
                    zwave_print(fs.data, fs.data_len);
                }
                if (fs.state_b != fs.B_PREAMP)
                {
                    std::cerr << "Unlock" << std::endl;
                }
                fs.state_b = fs.B_PREAMP;
                state = S_IDLE;
            }
        }
    }
    return 0;
}
