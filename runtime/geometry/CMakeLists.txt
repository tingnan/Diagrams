file(GLOB sources "*.cpp")
file(GLOB headers "*.h")
add_library(diaggeometry ${sources} ${headers})
target_link_libraries(diaggeometry ${POLY2TRI_LIBRARIES} ${CLIPPER_LIBRARIES})