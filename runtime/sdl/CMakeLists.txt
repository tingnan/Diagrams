file(GLOB sources "*.cpp") 
file(GLOB headers "*.h")
add_executable(main ${sources} ${headers})
target_link_libraries(main diagrender diagphysics diaggeometry diagutility ${JSONCPP_LIBRARIES} ${POLY2TRI_LIBRARIES} ${LIQUIDFUN_LIBRARIES} ${SDL2_LIBRARIES} ${OPENGL_LIBRARIES} ${NACL_LIBRARIES})