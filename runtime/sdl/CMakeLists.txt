FILE(GLOB sources "*.cpp") 
FILE(GLOB headers "*.h")
ADD_LIBRARY(diagrender ${sources} ${headers})
TARGET_LINK_LIBRARIES(diagrender diagphysics diaggeometry diagutility ${JSONLIBS} ${POLY2TRILIBS} ${LIQUIDFUN_LIBRARIES} ${OPENGL_LIBRARIES})