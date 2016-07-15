cmake_minimum_required(VERSION 3.2.0)
project(WavingZ)
set(CMAKE_CXX_STANDARD 11)
find_package(Boost 1.36 COMPONENTS program_options unit_test_framework REQUIRED)

include_directories(${Boost_INCLUDE_DIRS})

add_executable(wave-out wave-out.cpp wavingz.cpp)
add_executable(wave-in wave-in.cpp wavingz.cpp)

target_link_libraries(wave-in ${Boost_PROGRAM_OPTIONS_LIBRARIES})
target_link_libraries(wave-out ${Boost_PROGRAM_OPTIONS_LIBRARIES})

enable_testing()
add_subdirectory(test)
