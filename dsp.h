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
binomial_mult( std::vector< std::complex<double> > p )
{
    std::vector<std::complex<double>> a(p.size());
    for( size_t ii(0); ii != p.size(); ++ii)
    {
        for( size_t jj(ii); jj != 0; --jj)
        {
            a[jj] = p[ii] * a[jj-1];
        }
        a[0] += p[ii];
    }
    return a;
}

template< int ORDER >
std::vector<double> bcof_bwlp( double fcf )
{
    std::vector<std::complex<double>> rcof(ORDER);
    double theta = M_PI * fcf;
    double st = sin(theta);
    double ct = cos(theta);
    for( int k = 0; k < ORDER; ++k )
    {
        double parg = M_PI * (double)(2*k+1)/(double)(2*ORDER);
        double a = 1.0 + st * sin(parg);
        rcof[k].real(-ct / a);
        rcof[k].imag(-st * cos(parg) / a);
    }
    auto ddcof = binomial_mult( rcof );
    std::vector<double> dcof;
    dcof.push_back(1.0);
    for(int k(0); k!=ORDER; ++k)
    {
        dcof.push_back(ddcof[k].real());
    }
    return dcof;
}

template< int ORDER >
std::vector<double> bcof_bwhp( int n, double fcf )
{
    return( bcof_bwlp<ORDER>( fcf ) );
}

template< int ORDER >
double sf_bwlp( double fcf )
{
    int m, k;         // loop variables
    double omega;     // M_PI * fcf
    double fomega;    // function of omega
    double parg0;     // zeroth pole angle
    double sf;        // scaling factor

    omega = M_PI * fcf;
    fomega = sin(omega);
    parg0 = M_PI / (double)(2*ORDER);

    m = ORDER / 2;
    sf = 1.0;
    for( k = 0; k < ORDER/2; ++k )
        sf *= 1.0 + fomega * sin((double)(2*k+1)*parg0);

    fomega = sin(omega / 2.0);

    if( ORDER % 2 ) sf *= fomega + cos(omega / 2.0);
    sf = pow( fomega, ORDER ) / sf;

    return(sf);
}

template< int ORDER >
double sf_bwhp( double fcf )
{
    int m, k;         // loop variables
    double omega;     // M_PI * fcf
    double fomega;    // function of omega
    double parg0;     // zeroth pole angle
    double sf;        // scaling factor

    omega = M_PI * fcf;
    fomega = sin(omega);
    parg0 = M_PI / (double)(2*ORDER);

    m = ORDER / 2;
    sf = 1.0;
    for( k = 0; k < ORDER/2; ++k )
        sf *= 1.0 + fomega * sin((double)(2*k+1)*parg0);

    fomega = cos(omega / 2.0);

    if( ORDER % 2 ) sf *= fomega + sin(omega / 2.0);
    sf = pow( fomega, ORDER ) / sf;

    return sf;
}

template< int ORDER >
inline std::vector<double> ccof_bwlp( )
{
    std::vector<double> ccof(ORDER+1);
    ccof[0] = 1;
    ccof[1] = ORDER;
    int m = ORDER/2;
    for(int i=2; i <= m; ++i)
    {
        ccof[i] = (ORDER-i+1)*ccof[i-1]/i;
        ccof[ORDER-i]= ccof[i];
    }
    ccof[ORDER-1] = ORDER;
    ccof[ORDER] = 1;
    return ccof;
}

template< int ORDER >
inline std::vector<double> acof_bwlp( double fcf )
{
    auto ccof = ccof_bwlp<ORDER>( );
    double scale_factor = sf_bwlp<ORDER>(fcf);
    std::transform(ccof.begin(), ccof.end(), ccof.begin(),
                   std::bind1st(std::multiplies<double>(), scale_factor));
}

template< int ORDER >
inline std::vector<double> acof_bwhp( double fcf )
{
    auto ccof = ccof_bwlp<ORDER>( );
    for( int i = 1; i <= ORDER; i+=2) ccof[i] = -ccof[i];
    double scale_factor = sf_bwhp<ORDER>(fcf);
    std::transform(ccof.begin(), ccof.end(), ccof.begin(),
                   std::bind1st(std::multiplies<double>(), scale_factor));
    return ccof;
}

}

/// similar to octave-signal butter function, low-pass
template< int ORDER >
inline std::pair<std::array<double, ORDER + 1>, std::array<double, ORDER + 1>>
butter_lp(double sample_rate, double cutoff_freq)
{
    return std::make_pair(bcof_bwlp<ORDER>(cutoff_freq/sample_rate),
                          acof_bwlp<ORDER>(cutoff_freq/sample_rate));
}

/// similar to octave-signal butter function, high-pass
template< int ORDER >
inline std::pair<std::array<double, ORDER + 1>, std::array<double, ORDER + 1>>
butter_hp(int n, double sample_rate, double cutoff_freq)
{
    return std::make_pair(bcof_bwhp<ORDER>(cutoff_freq/sample_rate),
                          acof_bwhp<ORDER>(cutoff_freq/sample_rate));
}

/// Generic IIR filter simulator or specified ORDER
template <int ORDER>
struct iir_filter
{
    iir_filter(const std::array<double, ORDER+1> b,
               const std::array<double, ORDER+1> a)
      : b_m(b)
      , a_m(a)
      , xv_m(ORDER+1)
      , yv_m(ORDER+1)
    {
    }

    double operator()(double in)
    {
        xv_m.push_front(in);
        yv_m.push_front(0.0);
        yv_m[0] = std::inner_product(xv_m.begin(), xv_m.end(), b_m.begin(), 1.0) -
            std::inner_product(yv_m.begin(), yv_m.end(), a_m.begin(), 1.0);
        return yv_m[0];
    }

    std::array<double, ORDER + 1> b_m;
    std::array<double, ORDER + 1> a_m;
    boost::circular_buffer<double> xv_m;
    boost::circular_buffer<double> yv_m;
};
