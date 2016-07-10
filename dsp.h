/*
 *  Butterworth LP/HP IIR filter design
 *
 *  Copyright (C) 2016 Mirko Maischberger <mirko.maischberger@gmail.com>
 *
 *  Portions Copyright (C) 2014 Exstrom Laboratories LLC <stefan(AT)exstrom.com>
 *                              Longmont, CO 80503, USA
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  A copy of the GNU General Public License is available on the internet at:
 *  http://www.gnu.org/copyleft/gpl.html
 *
 */

#pragma once

#include <boost/circular_buffer.hpp>
#include <iostream>
#include <complex>
#include <cstdlib>
#include <cstdio>
#include <memory>
#include <vector>
#include <array>
#include <cmath>

namespace
{

inline std::vector<std::complex<double>>
binomial_mult(const std::vector<std::complex<double>>& p)
{
    std::vector<std::complex<double>> a(p.size(), 0.0);
    for (size_t ii(0); ii != p.size(); ++ii) {
        for (size_t jj(ii); jj != 0; --jj) {
            a[jj] += p[ii] * a[jj - 1];
        }
        a[0] += p[ii];
    }
    return a;
}

template <int ORDER>
std::array<double, ORDER + 1>
acof_bwlp(double fcf)
{
    std::vector<std::complex<double>> rcof(ORDER);
    double theta = M_PI * fcf;
    double st = sin(theta);
    double ct = cos(theta);
    for (int k = 0; k < ORDER; ++k) {
        double parg = M_PI * (2.0 * k + 1) / (2.0*ORDER);
        double a = 1.0 + st * sin(parg);
        rcof[k].real( -ct / a);
        rcof[k].imag( -st * cos(parg) / a);
    }
    auto ddcof = binomial_mult(rcof);

    std::array<double, ORDER + 1> dcof;
    dcof[0] = 1.0;
    for (int k(1); k != ORDER + 1; ++k) {
        dcof[k] = ddcof[k-1].real();
    }
    return dcof;
}

template <int ORDER>
std::array<double, ORDER + 1>
acof_bwhp(int n, double fcf)
{
    return acof_bwlp<ORDER>(fcf);
}

template <int ORDER>
double
sf_bwlp(double fcf)
{
    int m, k;      // loop variables
    double omega;  // M_PI * fcf
    double fomega; // function of omega
    double parg0;  // zeroth pole angle
    double sf;     // scaling factor

    omega = M_PI * fcf;
    fomega = sin(omega);
    parg0 = M_PI / (double)(2 * ORDER);

    m = ORDER / 2;
    sf = 1.0;
    for (k = 0; k < ORDER / 2; ++k)
        sf *= 1.0 + fomega * sin((double)(2 * k + 1) * parg0);

    fomega = sin(omega / 2.0);

    if (ORDER % 2)
        sf *= fomega + cos(omega / 2.0);
    sf = pow(fomega, ORDER) / sf;

    return (sf);
}

template <int ORDER>
double
sf_bwhp(double fcf)
{
    int m, k;      // loop variables
    double omega;  // M_PI * fcf
    double fomega; // function of omega
    double parg0;  // zeroth pole angle
    double sf;     // scaling factor

    omega = M_PI * fcf;
    fomega = sin(omega);
    parg0 = M_PI / (double)(2 * ORDER);

    m = ORDER / 2;
    sf = 1.0;
    for (k = 0; k < ORDER / 2; ++k)
        sf *= 1.0 + fomega * sin((double)(2 * k + 1) * parg0);

    fomega = cos(omega / 2.0);

    if (ORDER % 2)
        sf *= fomega + sin(omega / 2.0);
    sf = pow(fomega, ORDER) / sf;

    return sf;
}

template <int ORDER>
std::array<double, ORDER + 1>
ccof_bwlp()
{
    std::array<double, ORDER + 1> ccof;
    ccof[0] = 1;
    ccof[1] = ORDER;
    int m = ORDER / 2;
    for (int i = 2; i <= m; ++i) {
        ccof[i] = (ORDER - i + 1) * ccof[i - 1] / i;
        ccof[ORDER - i] = ccof[i];
    }
    ccof[ORDER - 1] = ORDER;
    ccof[ORDER] = 1;
    return ccof;
}

template <int ORDER>
std::array<double, ORDER + 1>
bcof_bwlp(double fcf)
{
    auto ccof = ccof_bwlp<ORDER>();
    double scale_factor = sf_bwlp<ORDER>(fcf);
    std::transform(ccof.begin(), ccof.end(), ccof.begin(),
                   std::bind1st(std::multiplies<double>(), scale_factor));
    return ccof;
}

template <int ORDER>
std::array<double, ORDER + 1>
bcof_bwhp(double fcf)
{
    auto ccof = ccof_bwlp<ORDER>();
    for (int i = 1; i <= ORDER; i += 2) ccof[i] = -ccof[i];
    double scale_factor = sf_bwhp<ORDER>(fcf);
    std::transform(ccof.begin(), ccof.end(), ccof.begin(),
                   std::bind1st(std::multiplies<double>(), scale_factor));
    return ccof;
}

} // anonymous namespace

///
/// An IIR Butterworth LP Filter
///
/// Similar to octave-signal butter function, low-pass
///
/// @param sample_rate The desired sample rate (=2*Nyquist)
/// @param cutoff_freq The -3dB cutoff frequency
///
/// @returns [b,a] coefficients ready to be used by the iir_filter class.
///
template <int ORDER>
std::pair<std::array<double, ORDER + 1>, std::array<double, ORDER + 1>>
butter_lp(double sample_rate, double cutoff_freq)
{
    return std::make_pair(bcof_bwlp<ORDER>(2.0 * cutoff_freq / sample_rate),
                          acof_bwlp<ORDER>(2.0 * cutoff_freq / sample_rate));
}

///
/// An IIR Butterworth HP Filter
///
/// Similar to octave-signal butter function, high-pass
///
/// @param sample_rate The desired sample rate (=2*Nyquist)
/// @param cutoff_freq The -3dB cutoff frequency
///
/// @returns [b,a] coefficients ready to be used by the iir_filter class.
///
template <int ORDER>
std::pair<std::array<double, ORDER + 1>, std::array<double, ORDER + 1>>
butter_hp(double sample_rate, double cutoff_freq)
{
    return std::make_pair(bcof_bwhp<ORDER>(2.0 * cutoff_freq / sample_rate),
                          acof_bwhp<ORDER>(2.0 * cutoff_freq / sample_rate));
}

///
/// Generic IIR filter simulator or specified ORDER
///
template <int ORDER>
struct iir_filter
{
    ///
    /// Creates the filter.
    ///
    /// Example:
    ///
    ///
    /// std::array<double, 2048> signal = {0.0, 1.0};
    /// std::array<double, 4> b, a;
    /// std::tie(b,a) = butter_lp<3>(2048000, 20480);
    /// iir_filter lp(b,a);
    ///
    /// for(double v: signal) v = lp(v);
    ///
    /// @param b Denominator of the filter
    /// @param a Numerator of the filter
    ///
    explicit
    iir_filter(const std::array<double, ORDER + 1>& b,
               const std::array<double, ORDER + 1>& a)
      : b_m(b)
      , a_m(a)
      , xv_m(ORDER + 1)
      , yv_m(ORDER + 1)
    {
        assert(a[0] == 1.0);
        for(size_t ii(0); ii!=b.size(); ++ii)
            assert(b[ii] == b[b.size()-ii-1]);
    }

    ///
    /// Feed the filter with samples.
    ///
    /// @param in The latest input to the filter
    /// @returns The filtered output
    ///
    double operator()(double in)
    {
        xv_m.push_front(in);
        yv_m.push_front(0.0);
        yv_m[0] =
          std::inner_product(xv_m.begin(), xv_m.end(), b_m.begin(), 1.0) -
          std::inner_product(yv_m.begin(), yv_m.end(), a_m.begin(), 1.0);
        return yv_m[0];
    }

  private:
    std::array<double, ORDER + 1> b_m;
    std::array<double, ORDER + 1> a_m;
    boost::circular_buffer<double> xv_m;
    boost::circular_buffer<double> yv_m;
};
