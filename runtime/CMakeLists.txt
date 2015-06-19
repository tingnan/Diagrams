IF(NACL)
SET(CMAKE_RANLIB "${NACL_TOOLCHAIN_ROOT}/bin/${NACL_CROSS_PREFIX}-ranlib" CACHE FILEPATH "Ranlib")
ENDIF()

PROJECT (diagrammarcpp)
CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

# need c++11 

IF(NOT NACL)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")
  INCLUDE_DIRECTORIES(/usr/include/jsoncpp /usr/include/eigen3 ./ ./third_party/poly2tri ./third_party/clipper)
  FIND_LIBRARY(JSONLIBS NAMES jsoncpp PATHS /usr/lib/x86_64-linux-gnu)

  # OpenGL
  find_package(OpenGL)
  find_package(GLEW)
  FIND_LIBRARY(GLFW_LIBRARIES glfw3 /opt/physics/lib)
  INCLUDE_DIRECTORIES(/opt/physics/include)

  # liquidfun
  FIND_LIBRARY(LIQUIDFUN_LIBRARIES liquidfun /opt/physics/lib)

  SET(ADDITIONAL_LIBS
  -lpthread
  -lXcursor
  -lXinerama
  -lXrandr
  -lXxf86vm
  -lXi
  )
  ADD_SUBDIRECTORY(third_party/poly2tri)
  SET(POLY2TRILIBS poly2tri)
  ADD_SUBDIRECTORY(third_party/clipper)
  SET(CLIPPERLIBS polyclipping)
  ADD_SUBDIRECTORY(utility)
  ADD_SUBDIRECTORY(geometry)
  ADD_SUBDIRECTORY(physics)
  ADD_SUBDIRECTORY(render)
  ADD_SUBDIRECTORY(testbed)
ELSE()
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=gnu++11")
  INCLUDE_DIRECTORIES("${CMAKE_FIND_ROOT_PATH}/usr/include/jsoncpp"
   "${CMAKE_FIND_ROOT_PATH}/usr/include/eigen3"
   "${CMAKE_FIND_ROOT_PATH}/usr/include/polyclipping"
   "${CMAKE_FIND_ROOT_PATH}/usr/include/poly2tri"
   ./)
  MESSAGE(STATUS "PATH: " ${NACL_TOOLCHAIN_ROOT}/${NACL_CROSS_PREFIX})
  FIND_LIBRARY(JSONLIBS NAMES jsoncpp)
  FIND_LIBRARY(CLIPPERLIBS NAMES polyclipping)
  FIND_LIBRARY(POLY2TRILIBS NAMES poly2tri)
  FIND_LIBRARY(LIQUIDFUN_LIBRARIES NAMES Box2D)
  ADD_SUBDIRECTORY(geometry)
  ADD_SUBDIRECTORY(physics)
  ADD_SUBDIRECTORY(utility)
  ADD_SUBDIRECTORY(nacl)
ENDIF()

IF(NOT NACL)
  ADD_SUBDIRECTORY(third_party/gtest-1.7.0)
  ENABLE_TESTING()
  INCLUDE_DIRECTORIES(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})
  ADD_SUBDIRECTORY(test/geometry)
ENDIF()
