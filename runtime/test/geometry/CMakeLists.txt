add_executable(test_coordinate test_coordinate.h test_coordinate.cpp)
target_link_libraries(test_coordinate gtest gtest_main diaggeometry)
add_test(NAME geometry-path COMMAND test_coordinate)
