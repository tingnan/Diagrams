file(GLOB sources "*.cpp") 
file(GLOB headers "*.h")
add_library(diaginterface ${sources} ${headers})
target_link_libraries(diaginterface diagrender diagphysics diaggeometry diagutility)
# ${JSON_LIBRARIES} ${POLY2TRI_LIBRARIES} ${LIQUIDFUN_LIBRARIES} ${OPENGL_LIBRARIES} ${FREETYPE_LIBRARIES}