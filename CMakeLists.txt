cmake_minimum_required(VERSION 3.1.0)
project(WavingZ)
set(CMAKE_CXX_STANDARD 11)
find_package(Boost 1.36 COMPONENTS program_options REQUIRED)

include_directories(${Boost_INCLUDE_DIRS})

add_executable(wave-out wave-out.cpp)
add_executable(wave-in wave-in.cpp)
add_executable(dsptest dsp.cc)
add_executable(iqplay play.cpp)

target_link_libraries(wave-in ${Boost_LIBRARIES})
target_link_libraries(wave-out ${Boost_LIBRARIES})
target_link_libraries(iqplay ${Boost_LIBRARIES})
