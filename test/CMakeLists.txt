add_executable(wavingz-test wavingz-test.cpp ../wavingz.cpp)
target_link_libraries(wavingz-test ${Boost_UNIT_TEST_FRAMEWORK_LIBRARIES})
add_test(NAME test-wavingz COMMAND $<TARGET_FILE:wavingz-test>)
