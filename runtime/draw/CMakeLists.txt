file(GLOB sources "*.cpp") 
file(GLOB headers "*.h")
add_library(diagrender ${sources} ${headers})
target_link_libraries(diagrender diaggeometry ${OPENGL_LIBRARIES} ${NACL_LIBRARIES})
