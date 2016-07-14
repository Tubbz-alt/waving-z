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
    std::tie(gain, b1, a1) = butter_lp<3>(SAMPLE_RATE, 80000 * 2.5);
    iir_filter<3> lp1(gain, b1, a1);
    auto lp2 = lp1;

    std::array<double, 4> a2, b2;
    std::tie(gain, b2, a2) = butter_lp<3>(SAMPLE_RATE, 40000 * 2.5);
    iir_filter<3> freq_filter(gain, b2, a2);

    std::array<double, 4> a3, b3;
    std::tie(gain, b3, a3) = butter_lp<3>(SAMPLE_RATE, 1200);
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
    size_t counter = 0;
    double omega_c = 0;
    for(;;) {
        double re, im;
        char ii, qq;
        if(!cin.get(ii) || !cin.get(qq)) break;
        if (unsigned_input)
        {
            re = double((uint8_t)ii)/127.0 - 1.0;
            im = double((uint8_t)qq)/127.0 - 1.0;
        }
        else
        {
            re = double(ii)/127.0;
            im = double(qq)/127.0;
        }
        assert(std::abs(re) <= 1.0);
        assert(std::abs(im) <= 1.0);
        double f = demod(lp1(re), lp2(im));
        double s = freq_filter(f);
        double lock_freq = lock_filter(f);
        boost::optional<bool> sample;
        // check for signal, adjust central freq, and get sample
        bool signal = fabs(lock_freq) > 0.015;
        if(signal)
        {
            if (samples_sm.idle()) omega_c = lock_freq;
            sample = (s - omega_c) < 0.0;
            if (samples_sm.preamble()) omega_c = 0.95 * omega_c + lock_freq * 0.05;
        }
        // process the sample with the state machine
        samples_sm.process(sample);
    }
    return 0;
}
