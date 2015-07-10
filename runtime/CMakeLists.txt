if(NACL)
set(CMAKE_RANLIB "${NACL_TOOLCHAIN_ROOT}/bin/${NACL_CROSS_PREFIX}-ranlib" CACHE FILEPATH "Ranlib")
endif()

project (diagrammarcpp)
cmake_minimum_required(VERSION 2.8)

# need c++11 
if(NOT NACL)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")
  include_directories(/usr/include/jsoncpp /usr/include/eigen3 ./ ./third_party/poly2tri ./third_party/clipper)
  include_directories(/opt/physics/include)

  find_library(LIQUIDFUN_LIBRARIES liquidfun /opt/physics/lib)
  find_library(VHACD_LIBRARIES VHACD_LIB /opt/physics/lib)
  
  find_package(OpenMP)
  if(OPENMP_FOUND)
    message("OPENMP FOUND")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
  endif()

  find_library(JSON_LIBRARIES NAMES jsoncpp PATHS /usr/lib/x86_64-linux-gnu)

  set(SDL 1)
  # OpenGL
  find_package(OpenGL)
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
    find_library(SDL_LIBRARIES NAMES SDL2)
    find_library(FREETYPE_LIBRARIES NAMES freetype)
  endif()  

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
  else()
    add_subdirectory(sdl)
  endif()
  add_subdirectory(testbed)
else()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=gnu++11")
  include_directories("${CMAKE_FIND_ROOT_PATH}/usr/include/jsoncpp"
   "${CMAKE_FIND_ROOT_PATH}/usr/include/eigen3"
   "${CMAKE_FIND_ROOT_PATH}/usr/include/polyclipping"
   "${CMAKE_FIND_ROOT_PATH}/usr/include/poly2tri"
   ./)
  message(STATUS "PATH: " ${NACL_TOOLCHAIN_ROOT}/${NACL_CROSS_PREFIX})
  find_library(JSON_LIBRARIES NAMES jsoncpp)
  find_library(CLIPPER_LIBRARIES NAMES polyclipping)
  find_library(POLY2TRI_LIBRARIES NAMES poly2tri)
  find_library(LIQUIDFUN_LIBRARIES NAMES Box2D)
  add_subdirectory(geometry)
  add_subdirectory(physics)
  add_subdirectory(utility)
  add_subdirectory(nacl)
endif()

if(NOT NACL)
  add_subdirectory(third_party/gtest-1.7.0)
  enable_testing()
  include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})
  add_subdirectory(test/geometry)
endif()
