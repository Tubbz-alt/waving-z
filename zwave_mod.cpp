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

#include <boost/circular_buffer.hpp>


const double SAMPLE_RATE = 2.048e6;
const double baud = 40.96e3;
const int Ts = SAMPLE_RATE / baud;

const char preamble = 0x55;
const char SOF = 0xF0;
const char HomeID0 = 0xd6;
const char HomeID1 = 0xb2;
const char HomeID2 = 0x62;
const char HomeID3 = 0x08;
const char SourceID = 0x01;
const char FC0 = 0x41;
const char FC1 = 0x02;
const char DestID = 0x03;

std::vector<char> buffer;

/// CRC8 Checksum calculator
template <typename It>
char
checksum(It begin, It end)
{
    return std::accumulate(begin, end, 0xff, std::bit_xor<char>());
}

/// Generic IIr filter implementation
template <int ORDER>
struct iir_filter
{
    iir_filter(const std::array<double, ORDER + 1> numerator,
               const std::array<double, ORDER> denominator)
      : a(numerator)
      , b(denominator)
      , xv(numerator.size())
      , yv(denominator.size())
    {
    }

    double operator()(double in)
    {
        xv.push_front(in);
        double y0 = std::inner_product(xv.begin(), xv.end(), a.begin(), 1.0) -
                    std::inner_product(yv.begin(), yv.end(), b.begin(), 1.0);
        yv.push_front(y0);
        return y0;
    }

    std::array<double, ORDER + 1> a;
    std::array<double, ORDER> b;
    boost::circular_buffer<double> xv;
    boost::circular_buffer<double> yv;
};

using namespace std;

void
print_iq(double i, double q)
{
    const double A = 126.0;
    const double offset = 0.0;
    std::cout << (int8_t)(i*A + offset) << (int8_t)(q*A + offset);
};

// 40000bps, NZR, Freq-offset=0, Separation=40KHz
int
main()
{
    for (int ii(0); ii < 16; ++ii)
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
    buffer.push_back(0x00);
    buffer.push_back(0x99);
    buffer.push_back(checksum(buffer.begin() + skip, buffer.end()));

    int tt = 0;

    iir_filter<1> lp1({ 1.0 }, { });
    auto lp2 = lp1;

    for (int ii(0); ii != 4000000; ++ii)
    {
        double i = lp1(0.0);
        double q = lp2(0.0);
        print_iq(i, q);
    }

    for (char ch : buffer)
    {
        for (int ii(0); ii != 8; ++ii)
        {
            double f_shift = 20.48e3 * ((ch << (ii) & 0x80) ? 2.40 : 0.4);
            for (int kk(0); kk != Ts; ++kk)
            {
                double i = lp1(sin(2.0 * M_PI * f_shift * (double)tt / SAMPLE_RATE));
                double q = lp2(cos(2.0 * M_PI * f_shift * (double)tt / SAMPLE_RATE));
                print_iq(i, q);
                ++tt;
            }
        }
    }
    for (int ii(0); ii != 4000000; ++ii)
    {
        double i = lp1(0.0);
        double q = lp2(0.0);
        print_iq(i, q);
    }
}
