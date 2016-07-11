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
#include <array>
#include <iostream>

int main()
{
    std::array<double, 2048> signal = { 1.0 };
    double gain;
    std::array<double, 6> b, a;
    std::tie(gain, b, a) = butter_lp<5>(2048000, 20480);
    iir_filter<5> lp(gain, b, a);
    int count = 0;
    for(double v: signal) std::cout << count++ << " " << v << " " << lp(v) << std::endl;
    return 0;
}
