file(GLOB sources "*.cpp")
file(GLOB headers "*.h")
add_library(diagutility ${sources} ${headers})
target_link_libraries(diagutility diaggeometry ${JSON_LIBRARIES})