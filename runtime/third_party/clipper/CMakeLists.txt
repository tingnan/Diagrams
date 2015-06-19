include_directories(./polyclipping)
file(GLOB_RECURSE sources ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)
add_library(polyclipping ${sources})
