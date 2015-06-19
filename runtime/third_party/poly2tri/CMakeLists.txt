include_directories(./poly2tri ./poly2tri/common ./poly2tri/sweep)
file(GLOB_RECURSE sources ${CMAKE_CURRENT_SOURCE_DIR}/poly2tri/*.cc)
add_library(poly2tri ${sources})


