#include <cmath>
#include <vector>
#include <iostream>
#include <algorithm>
#include <functional>

const double SAMPLE_RATE = 2048000;
const double baud = 40960;
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

using namespace std;

// 40000bps, NZR, Freq-offset=0, Separation=40KHz
int
main()
{
    for (int ii(0); ii < 18; ++ii)
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
    double phase = 0;

    for (int ii(0); ii != 5000; ++ii)
    {
        std::cout << char(0x7f);
    }

    for (char ch : buffer)
    {
        for (int ii(0); ii != 8; ++ii)
        {
            double f_shift = 20480.0 * ((ch << (ii) & 0x80) ? 4.0 : 2.0);
            for (int kk(0); kk != Ts; ++kk)
            {
                std::cout << (char)(sin(2.0 * (double)tt * M_PI * f_shift / SAMPLE_RATE + phase) * 100.0 + 127.0);
                std::cout << (char)(cos(2.0 * (double)tt * M_PI * f_shift / SAMPLE_RATE + phase) * 100.0 + 127.0);
                ++tt;
            }
            // phase += ((ch >> ii & 0x01) ? -1.0 : 1.0) * M_PI * h;
        }
    }

    for (int ii(0); ii != 5000; ++ii)
    {
        std::cout << char(0x7f);
    }
}
