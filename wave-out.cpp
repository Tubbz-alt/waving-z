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

#include <cmath>
#include <vector>
#include <iostream>
#include <algorithm>
#include <functional>
#include <cstdio>
#include <cstdint>
#include <complex>
#include <unistd.h>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <cassert>

const double SAMPLE_RATE = 2.000e6;
const double baud = 40.00e3;
const int Ts = SAMPLE_RATE / baud;

const char preamble = 0x55;
const char SOF = 0xF0;
const char HomeID0 = 0xd6;
const char HomeID1 = 0xb2;
const char HomeID2 = 0x62;
const char HomeID3 = 0x08;
const char SourceID = 0x01;
const char FC0 = 0x41;
const char FC1 = 0x0a;
const char DestID = 0x03;

using namespace std;

void
print_iq(double i, double q)
{
    const double A = 110.0;
    const double offset = 0.0;
    if(std::abs(i*A) > 127.0 || std::abs(q*A) > 127.0)
    {
        throw std::runtime_error("Value too big!");
    }
    std::cout << (int8_t)(i*A + offset) << (int8_t)(q*A + offset);
};

// 40000bps, NZR, Freq-offset=0, Separation=40KHz
int
main()
{
    std::vector<char> buffer;

    for (int ii(0); ii < 20; ++ii)
    {
        buffer.push_back(preamble);
    }
    buffer.push_back(SOF);
    size_t skip = buffer.size();
    buffer.push_back(HomeID0);
    buffer.push_back(HomeID1);
    buffer.push_back(HomeID2);
    buffer.push_back(HomeID3);
    buffer.push_back(SourceID);
    buffer.push_back(FC0);
    buffer.push_back(FC1);
    buffer.push_back(13); // length
    buffer.push_back(DestID);
    buffer.push_back(37);
    buffer.push_back(0x01);
    buffer.push_back(0xff);
    buffer.push_back(0x6e);
    buffer.push_back(checksum(buffer.begin() + skip, buffer.end()));

    int tt = 0;

    double dfreq = 20.00e3;

    //double gain;
    //std::array<double, 3> b, a;
    //std::tie(gain, b, a) = butter_lp<2>(SAMPLE_RATE, 5.0 * dfreq);
    //iir_filter<2> lp1(gain, b, a);
    //auto lp2 = lp1;

#define lp1
#define lp2

    for (int ii(0); ii != 2 * SAMPLE_RATE; ++ii) {
        double i = lp1(0.0);
        double q = lp2(0.0);
        print_iq(i, q);
    }

    for (char ch : buffer) {
        for (int ii(0); ii != 8; ++ii) {
            double f_shift = dfreq * ((ch << (ii)&0x80) ? 2.5 : 0.5);
            for (int kk(0); kk != Ts; ++kk) {
                double i =
                  lp1(sin(2.0 * M_PI * f_shift * (double)tt / SAMPLE_RATE));
                double q =
                  lp2(cos(2.0 * M_PI * f_shift * (double)tt / SAMPLE_RATE));
                print_iq(i, q);
                ++tt;
            }
        }
    }
    for (int ii(0); ii != 2 * SAMPLE_RATE; ++ii) {
        double i = lp1(0.0);
        double q = lp2(0.0);
        print_iq(i, q);
    }
}
