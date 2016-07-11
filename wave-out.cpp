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
// FSK @40000bps, NZR, Separation=40KHz

#include "dsp.h"
#include "wavingz.h"
#include <boost/program_options.hpp>

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
#include <sstream>

const double SAMPLE_RATE = 2e6;
const double baud = 40e3;
const int Ts = SAMPLE_RATE / baud;

const char preamble = 0x55;
const char SOF = 0xF0;
const char SourceID = 0x01;
const char FC0 = 0x41;
const char FC1 = 0x05;

using namespace std;
namespace po = boost::program_options;

void
print_iq(double i, double q)
{
    const double A = 120.0;
    const double offset = 0.0;
    if(std::abs(i*A) > 127.0 || std::abs(q*A) > 127.0)
    {
        throw std::runtime_error("Value too big!");
    }
    std::cout << (int8_t)(i*A + offset) << (int8_t)(q*A + offset);
};

// Disable the filtering
#define lp1(x) (x)
#define lp2(x) (x)

int
main(int argc, char* argv[])
{

    std::string homeid_hex;
    int destid;

    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("homeid", po::value<std::string>(&homeid_hex), "home id in (hex format without 0x)")
        ("destid", po::value<int>(&destid), "destination id (decimal)")
        ("off", "send off packet (default on)")
       ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help")) {
        cout << desc << "\n";
        return 1;
    }

    uint32_t homeid;
    if (!vm.count("homeid")) {
        cout << "HomeID is mandatory." << std::endl;
        return -1;
    } else {
        std::stringstream interpreter;
        interpreter << std::hex << vm["homeid"].as<std::string>();
        interpreter >> homeid;
    }

    if (!vm.count("destid")) {
        cout << "DestinationID is mandatory." << std::endl;
        return -1;
    }

    std::cerr << std::hex << homeid << " " << (int)destid << std::endl;

    std::vector<int8_t> buffer;
    for (int ii(0); ii < 20; ++ii)
    {
        buffer.push_back(preamble);
    }
    buffer.push_back(SOF);
    size_t crc_skip = buffer.size();
    buffer.push_back(homeid >> 24);
    buffer.push_back(homeid >> 16 & 0xff);
    buffer.push_back(homeid >> 8 & 0xff);
    buffer.push_back(homeid & 0xff);
    buffer.push_back(SourceID);
    buffer.push_back(FC0);
    buffer.push_back(FC1);
    buffer.push_back(13); // length
    buffer.push_back((char)destid);
    buffer.push_back(37);
    buffer.push_back(0x01);
    if( vm.count("off"))
    {
        buffer.push_back(0x00);
        buffer.push_back(0x9e);
    }
    else
    {
        buffer.push_back(0xff);
        buffer.push_back(0x62);
    }
    buffer.push_back(wavingz::checksum(buffer.begin() + crc_skip, buffer.end()));

    int tt = 0;

    double dfreq = 20e3;

    for (int ii(0); ii != 2 * SAMPLE_RATE; ++ii) {
        double i = lp1(0.0);
        double q = lp2(0.0);
        print_iq(i, q);
    }

    for (uint8_t ch : buffer) {
        for (int ii(0); ii != 8; ++ii) {
            double f_shift = dfreq * ( ((ch << ii) & 0x80) ? 2.5 : 0.5);
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
