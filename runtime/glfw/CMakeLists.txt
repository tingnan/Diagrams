file(GLOB sources "*.cpp") 
file(GLOB headers "*.h")
add_library(diaginterface ${sources} ${headers})
target_link_libraries(diagrender diagutility diagphysics diaggeometry)