FILE(GLOB sources "*.cpp") 
FILE(GLOB headers "*.h")
ADD_LIBRARY(diaginterface ${sources} ${headers})
TARGET_LINK_LIBRARIES(diaginterface diagrender diagphysics diaggeometry diagutility)
# ${JSON_LIBRARIES} ${POLY2TRI_LIBRARIES} ${LIQUIDFUN_LIBRARIES} ${OPENGL_LIBRARIES} ${FREETYPE_LIBRARIES}