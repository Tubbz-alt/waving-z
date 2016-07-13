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
// Based on FSK demodulator from https://github.com/andersesbensen/rtl-zwave
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

#include <boost/optional.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/program_options.hpp>

using namespace std;
namespace po = boost::program_options;

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

    atan_fm_demodulator demod;

    double gain;
    std::array<double, 4> a1, b1;
    std::tie(gain, b1, a1) = butter_lp<3>(SAMPLE_RATE, 2.0 * 20000 * 2.5);
    iir_filter<3> lp1(gain, b1, a1);
    auto lp2 = lp1;

    std::array<double, 4> a2, b2;
    std::tie(gain, b2, a2) = butter_lp<3>(SAMPLE_RATE, 100000);
    iir_filter<3> freq_filter(gain, b2, a2);

    std::array<double, 4> a3, b3;
    std::tie(gain, b3, a3) = butter_lp<3>(SAMPLE_RATE, 1000);
    iir_filter<3> lock_filter(gain, b3, a3);

    struct process_wavingz {
        void operator()(uint8_t* begin, uint8_t*end)
        {
            wavingz::zwave_print(begin, end);
        }
    } wave_callback;

    // the symbols will be converted to a payload (or fail)
    wavingz::demod_sm::symbol_sm_t symbols_sm(wave_callback);
    // the samples will be converted to symbols
    wavingz::demod_sm::sample_sm_t samples_sm(SAMPLE_RATE, symbols_sm);

    while (!cin.eof()) {
        double re, im;
        if (unsigned_input)
        {
            uint8_t ii, qq;
            cin >> ii >> qq;
            re = lp1(double(ii)/127.0 - 1.0);
            im = lp2(double(qq)/127.0 - 1.0);
        }
        else
        {
            int8_t ii, qq;
            cin >> ii >> qq;
            re = lp1(double((int8_t)ii)/127.0);
            im = lp2(double((int8_t)qq)/127.0);
        }

        double f = demod(re, im);
        double s = freq_filter(f);
        double lock_freq = lock_filter(f);
        double omega_c = lock_freq;
        boost::optional<bool> sample;

        // check for signal, adjust central freq, and get sample
        if(fabs(lock_freq) > 0.01)
        {
            if (sample == boost::none) omega_c = lock_freq;
            if (!samples_sm.bitlock()) omega_c = 0.95 * omega_c + 0.05 * lock_freq;
            sample = (s - omega_c) < 0;
        }
        else
        {
            sample = boost::none;
        }

        // process the sample with the state machine
        samples_sm.process(sample);
    }
    return 0;
}
