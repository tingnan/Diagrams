FILE(GLOB sources "*.cpp") 
FILE(GLOB headers "*.h")
ADD_LIBRARY(diagrender ${sources} ${headers})
TARGET_LINK_LIBRARIES(diagrender diagutility diagphysics diaggeometry)