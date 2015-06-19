file(GLOB sources "*.cpp")
file(GLOB headers "*.h")
add_library(diagphysics ${sources} ${headers})
target_link_libraries(diagphysics diagutility ${LIQUIDFUN_LIBRARIES})
