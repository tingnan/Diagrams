# Cross-compiling requires CMake 2.6 or newer. To cross-compile, first modify
# this file to set the proper settings and paths. Then use it from build/ like:
# cmake .. -DCMAKE_TOOLCHAIN_FILE=../XCompile.txt \
#          -DCMAKE_INSTALL_PREFIX=/usr/mingw32/mingw
# If you already have a toolchain file setup, you may use that instead of this
# file.

set(NACL 1)

# find the SDK root
if("$ENV{NACL_SDK_ROOT}" STREQUAL "")
  MESSAGE(STATUS "NACL_SDK_ROOT not found! Set NACL_PATH to HOME/nacl_sdk/pepper_canary")
  set(NACL_SDK_ROOT "$ENV{HOME}/nacl_sdk/pepper_canary")
else()
  set(NACL_SDK_ROOT "$ENV{NACL_SDK_ROOT}")
endif()

# set system architecture
# can be x86_64 i686 arm or pnacl or use
# set(NACL_ARCH ${CMAKE_HOST_SYSTEM_PROCESSOR})
set(NACL_ARCH "pnacl")
# the name of the target operating system
set(CMAKE_SYSTEM_NAME Linux)

# get the toolchain
set(NACL_TOOLCHAIN_ROOT "${NACL_SDK_ROOT}/toolchain/linux_pnacl")
# set the prefix for compiler tools
if(${NACL_ARCH} STREQUAL "pnacl")
  set(NACL_CROSS_PREFIX "pnacl")
else()
  set(NACL_CROSS_PREFIX "${NACL_ARCH}-nacl")
endif()

# compilers to use for C and C++
set(CMAKE_AR "${NACL_TOOLCHAIN_ROOT}/bin/${NACL_CROSS_PREFIX}-ar" CACHE FILEPATH "Archiver")
set(CMAKE_LINKER "${NACL_TOOLCHAIN_ROOT}/bin/${NACL_CROSS_PREFIX}-ld" CACHE FILEPATH "Linker")
set(CMAKE_RANLIB "${NACL_TOOLCHAIN_ROOT}/bin/${NACL_CROSS_PREFIX}-ranlib" CACHE FILEPATH "Ranlib")
set(CMAKE_C_COMPILER "${NACL_TOOLCHAIN_ROOT}/bin/${NACL_CROSS_PREFIX}-clang")
set(CMAKE_CXX_COMPILER "${NACL_TOOLCHAIN_ROOT}/bin/${NACL_CROSS_PREFIX}-clang++")

if(${NACL_ARCH} STREQUAL "pnacl")
  set(NACL_SDK_LIBDIR "/lib/${NACL_ARCH}/Release")
  set(CMAKE_FIND_ROOT_PATH ${NACL_SDK_ROOT} ${NACL_TOOLCHAIN_ROOT}/le32-nacl)
else()
  set(NACL_SDK_LIBDIR "/lib/clang-newlib_${NACL_ARCH}/Release")
  set(CMAKE_FIND_ROOT_PATH ${NACL_SDK_ROOT} ${NACL_TOOLCHAIN_ROOT}/${NACL_CROSS_PREFIX})
endif()

# Reference CMAKE_TOOLCHAIN_FILE and CMAKE_PREFIX_PATH here to avoid
# cmake warnings:
# http://public.kitware.com/pipermail/cmake/2013-February/053476.html
set(DUMMY_VALUE ${CMAKE_TOOLCHAIN_FILE} ${CMAKE_PREFIX_PATH})


set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
# nacl abi says 32bits little endian
set(CMAKE_SIZEOF_VOID_P 4)