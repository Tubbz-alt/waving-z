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

    struct process_wavingz {
        void operator()(uint8_t* begin, uint8_t*end)
        {
            wavingz::zwave_print(begin, end);
        }
    } wave_callback;

    const int sample_rate = 2048000;
    wavingz::demod::demod_nrz wavein(sample_rate, wave_callback);
    size_t counter = 0;
    double omega_c = 0;
    for(;;) {
        std::complex<double> iq;
        char ii, qq;
        if(!cin.get(ii) || !cin.get(qq)) break;
        if (unsigned_input)
        {
            iq.real(double((uint8_t)ii)/127.0 - 1.0);
            iq.imag(double((uint8_t)qq) / 127.0 - 1.0);
        }
        else
        {
            iq.real(double(ii)/127.0);
            iq.imag(double(qq)/127.0);
        }
        assert(std::abs(iq) <= 1.0);
        wavein(iq);
    }
    return 0;
}
