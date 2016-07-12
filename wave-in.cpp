//
// Heavily based on https://github.com/andersesbensen/rtl-zwave
//
// This file is part of WavingZ.
//

#include "dsp.h"
#include "wavingz.h"

#include <cstdio>
#include <cstdint>
#include <complex>
#include <unistd.h>
#include <iostream>
#include <cassert>
#include <functional>
#include <algorithm>

#include <boost/circular_buffer.hpp>
#include <boost/program_options.hpp>

using namespace std;
namespace po = boost::program_options;

struct frame_state
{
    unsigned int bit_count;
    unsigned int data_len;
    unsigned char data[64];
    bool last_bit;
    int b_cnt;
    enum
    {
        B_PREAMB,
        B_SOF0,
        B_SOF1,
        B_DATA
    } state_b;
} fs;

enum
{
    S_IDLE,
    S_PREAMB,
    S_BITLOCK
} state = S_IDLE;

int
main(int argc, char** argv)
{
    po::options_description desc("WavingZ - Wave-in options");
    desc.add_options()
        ("help", "Produce this help message")
        ("unsigned", "Use unsigned8 (RTL-SDR) instead of signed8 (HackRF One)")
        ("debug", "Produce debug output")
       ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help")) {
        cout << desc << "\n";
        return 1;
    }

    bool unsigned_input = vm.count("unsigned");

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
    size_t frame_start_num;
    size_t f_num = 0; // Z-wave frame number
    int rec_ptr = 0;
    int frame_start;

    atan_fm_demodulator demod;

    double gain;
    std::array<double, 4> a1, b1;
    std::tie(gain, b1, a1) = butter_lp<3>(SAMPLE_RATE, 2.0 * 20000 * 2.5);
    iir_filter<3> lp1(gain, b1, a1);
    auto lp2 = lp1;

    std::array<double, 4> a2, b2;
    std::tie(gain, b2, a2) = butter_lp<3>(SAMPLE_RATE, 102400);
    iir_filter<3> freq_filter(gain, b2, a2);

    std::array<double, 4> a3, b3;
    std::tie(gain, b3, a3) = butter_lp<3>(SAMPLE_RATE, 10240);
    iir_filter<3> lock_filter(gain, b3, a3);

    while (!feof(stdin)) {
        uint8_t g[1024];
        if (fread(g, 1024, 1, stdin) != 1)
        {
            continue;
        };

        for (int i = 0; i < 1024; i += 2) {

            double re;
            double im;
            if (unsigned_input)
            {
                re = double(g[i])/127.0 - 1.0;
                im = double(g[i + 1])/127.0 - 1.0;
            }
            else
            {
                re = double((int8_t)g[i])/127.0;
                im = double((int8_t)g[i + 1])/127.0;
            }
            s_num++; // sample_number (* 2 == byte offset)

            double re2 = lp1(re);
            double im2 = lp2(im);
            f = demod(re2, im2);
            s = freq_filter(f);
            lock = lock_filter(f);
            hasSignal = fabs(lock) > 0.001;

            if (hasSignal)
            {
                bool logic = (s - wc) < 0;
                if (state == S_IDLE)
                {
                    frame_start_num = s_num; // mark frame start sample for later
                    state = S_PREAMB;
                    pre_cnt = 0;
                    pre_len = 0;
                    frame_start = rec_ptr;
                    wc = lock;
                }
                else if (state == S_PREAMB)
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
                            fs.state_b = fs.B_PREAMB;
                            fs.last_bit = not logic;
                            bit_len = double(pre_len) / (pre_cnt - lead_in - 1);
                            bit_cnt = 3 * bit_len / 4.0;
                            dr = SAMPLE_RATE / bit_len;
                            msc = dr < 15e3; // Should we use manchester
                                             // encoding
                            if( dr > 42e3 )
                            {
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
                        // Sub state machine
                        if (fs.state_b == fs.B_PREAMB)
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
                                }
                            }
                            else
                            {
                                // SOF0 error (bit_len);
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
                    wavingz::zwave_print(fs.data, fs.data + fs.data_len, frame_start_num, s_num);
                }
                fs.state_b = fs.B_PREAMB;
                state = S_IDLE;
            }
            if(vm.count("debug"))
                std::cerr << s_num << " " << re << " " << im << " " << hasSignal << " " << last_logic << " " << f_num << std::endl;
        }
    }
    return 0;
}
