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

#include <iostream>

#include <boost/program_options.hpp>

using namespace std;
namespace po = boost::program_options;

int
main(int argc, char* argv[])
{
    size_t start_sample;
    size_t end_sample;

    po::options_description desc("WavingZ - Wave-in options");
    desc.add_options()
        ("help", "Produce this help message")
        ("start", po::value<size_t>(&start_sample), "Reproduce IQ file starting from sample")
        ("end", po::value<size_t>(&end_sample), "Reproduce IQ file up to end sample")
       ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help"))
    {
        cout << desc << "\n";
        return 1;
    }

    cin.ignore(start_sample * 2);

    if (!cin.good())
        return -1;

    for (size_t cnt(start_sample); cnt != end_sample; ++cnt) {
        int i = cin.get();
        if (i == EOF)
            return -1;
        int q = cin.get();
        if (q == EOF)
            return -1;
        cout << (char)i << (char)q;
    }

    return 0;
}
