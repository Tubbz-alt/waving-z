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

const uint8_t SourceID = 0x01; // The source id is usually 1
const uint8_t FC0 = 0x41;      // Frame control byte 0
const uint8_t FC1 = 0x05;      // Frame control byte 1

using namespace std;
namespace po = boost::program_options;


int
main(int argc, char* argv[])
{
    std::string homeid_hex;
    int destid;
    std::string payload;

    po::options_description desc("WavingZ - Wave-out options");
    desc.add_options()
        ("help", "Produce this help message")
        ("homeid", po::value<std::string>(&homeid_hex), "The user home id in (hex format without 0x)")
        ("destid", po::value<int>(&destid), "The desired destination id (decimal)")
        ("packet", po::value<std::string>(&payload), "Payload to send (hex format without 0x)")
        ("unsigned", "Produce uint8 output instead if int8")
       ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help")) {
        cerr << desc << "\n";
        return 1;
    }

    uint32_t homeid;
    if (!vm.count("homeid")) {
        cerr << "HomeID is mandatory." << std::endl;
        return -1;
    } else {
        std::stringstream interpreter;
        interpreter << std::hex << vm["homeid"].as<std::string>();
        interpreter >> homeid;
    }

    if (!vm.count("destid")) {
        cerr << "DestinationID is mandatory." << std::endl;
        return -1;
    }

    std::cerr << std::hex << homeid << " " << destid << std::endl;

    // Create wavingz buffer
    std::vector<uint8_t> buffer;
    buffer.push_back(homeid >> 24);
    buffer.push_back(homeid >> 16 & 0xff);
    buffer.push_back(homeid >> 8 & 0xff);
    buffer.push_back(homeid & 0xff);
    buffer.push_back(SourceID);
    buffer.push_back(FC0);
    buffer.push_back(FC1);
    buffer.push_back(13); // length
    buffer.push_back((uint8_t)destid);
    buffer.push_back(37);
    buffer.push_back(0x01);
    buffer.push_back(0x00);
    buffer.push_back(0x9e);
    buffer.push_back(wavingz::checksum(buffer.begin(), buffer.end()));

    // encode and output wavingz buffer
    if(vm.count("unsigned"))
    {
        auto complex_bytes = wavingz::encode<uint8_t>(buffer.begin(), buffer.end());
        for(auto pair: complex_bytes)
            std::cout << pair.first << pair.second;
    }
    else
    {
        auto complex_bytes = wavingz::encode<int8_t>(buffer.begin(), buffer.end());
        for(auto pair: complex_bytes)
            std::cout << pair.first << pair.second;
    }

    return 0;
}
