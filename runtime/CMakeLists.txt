if(NACL)
set(CMAKE_RANLIB "${NACL_TOOLCHAIN_ROOT}/bin/${NACL_CROSS_PREFIX}-ranlib" CACHE FILEPATH "Ranlib")
endif()

project (diagrammarcpp)
cmake_minimum_required(VERSION 2.8)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmake/modules)

# need c++11 
if(NOT NACL)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")

  find_package(Eigen3)  
  include_directories(${EIGEN3_INCLUDE_DIRS})
  find_package(SDL2)
  include_directories(${SDL2_INCLUDE_DIRS})
  find_package(JsonCpp)
  include_directories(${JSONCPP_INCLUDE_DIRS})
  find_package(Box2D)
  include_directories(${LIQUIDFUN_INCLUDE_DIRS})
  find_package(OpenGL)

  set(SDL 1)  
  if(NOT SDL)
    find_package(GLEW)
    find_library(GLFW_LIBRARIES glfw3 /opt/physics/lib)
    set(ADDITIONAL_LIBRARIES
    -lpthread
    -lXcursor
    -lXinerama
    -lXrandr
    -lXxf86vm
    -lXi
    )
  else()
    find_library(FREETYPE_LIBRARIES NAMES freetype)
  endif()  
  include_directories(./ third_party/poly2tri third_party/clipper)
  add_subdirectory(third_party/poly2tri)
  set(POLY2TRI_LIBRARIES poly2tri)
  add_subdirectory(third_party/clipper)
  set(CLIPPER_LIBRARIES polyclipping)
  add_subdirectory(utility)
  add_subdirectory(geometry)
  add_subdirectory(physics)
  add_subdirectory(gl)
  if(NOT SDL)
    add_subdirectory(glfw)
    add_subdirectory(testbed)
  else()
    add_subdirectory(sdl)
  endif()

else()
  
  IF(${NACL_ARCH} STREQUAL "pnacl")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=gnu++11")
  else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")
  endif()
  
  include_directories(./ )
  find_package(Eigen3)  
  include_directories(${EIGEN3_INCLUDE_DIRS})
  find_package(SDL2)
  include_directories(${SDL2_INCLUDE_DIRS})
  find_package(NaCl)
  find_package(JsonCpp)
  include_directories(${JSONCPP_INCLUDE_DIRS})
  find_package(Poly2Tri)
  include_directories(${POLY2TRI_INCLUDE_DIRS})  
  find_package(Clipper)
  include_directories(${CLIPPER_INCLUDE_DIRS})
  find_package(Box2D)
  include_directories(${LIQUIDFUN_INCLUDE_DIRS})
  find_package(GLES2)
  include_directories(${GLES2_INCLUDE_DIRS})

  add_subdirectory(geometry)
  add_subdirectory(physics)
  add_subdirectory(utility)
  add_subdirectory(gl)
  add_subdirectory(sdl)

endif()

if(NOT NACL)
  add_subdirectory(third_party/gtest-1.7.0)
  enable_testing()
  include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})
  add_subdirectory(test/geometry)
endif()
