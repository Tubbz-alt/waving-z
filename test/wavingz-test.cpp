#include "../dsp.h"

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE MyTest
#include <boost/test/unit_test.hpp>

#define CHECK_CLOSE_COLLECTION(aa, bb, tolerance) {     \
    using std::distance; \
    using std::begin; \
    using std::end; \
    auto a = begin(aa), ae = end(aa); \
    auto b = begin(bb); \
    BOOST_REQUIRE_EQUAL(distance(a, ae), distance(b, end(bb))); \
    for(; a != ae; ++a, ++b) { \
        BOOST_CHECK_CLOSE(*a, *b, tolerance); \
    } \
}

BOOST_AUTO_TEST_CASE(test_butter)
{
    // generated with octave signal package
    // > pkg load signal
    // > [b,a] = butter(6, 2*80000/2048000)
    double expected_gain = 2.18780328998614e-06;
    std::array<double, 7> expected_b({ 1, 6, 15, 20, 15, 6, 1 });
    std::array<double, 7> expected_a(
      { 1, -5.052163948341672, 10.699633740567215, -12.151435255115082,
        7.801326239249508, -2.683448745937741, 0.386227988988330 });

    double actual_gain;
    std::array<double, 7 > actual_b;
    std::array<double, 7 > actual_a;

    std::tie(actual_gain, actual_b, actual_a) = butter_lp<6>(2048000, 80000);

    BOOST_CHECK_CLOSE(expected_gain, actual_gain, 1e-12);
    CHECK_CLOSE_COLLECTION(expected_b, actual_b, 1e-12);
    CHECK_CLOSE_COLLECTION(expected_a, actual_a, 1e-12);
}

BOOST_AUTO_TEST_CASE(test_iir_filter)
{
    std::array<double, 7> signal = { 1.0 };
    // generated with octave signal package
    // >> pkg load signal;
    // >> [b,a] = butter(6, 2*40000/2048000);
    // >> v = impz(b, a, 7)
    std::array<double, 7> response = {
        4.24141395075581e-08, 4.88861571352154e-07, 2.79723456125130e-06,
        1.07425029331562e-05, 3.15672364704611e-05, 7.65594176229739e-05,
        1.60949149809997e-04
    };
    double gain;
    std::array<double, 7 > b;
    std::array<double, 7 > a;

    std::tie(gain, b, a) = butter_lp<6>(2048000, 40000);
    iir_filter<6> lp(gain, b, a);
    size_t counter = 0;
    for(double v: signal)
    {
        BOOST_CHECK_CLOSE(response[counter], lp(signal[counter]), 1e-12);
        counter++;
    }
}
