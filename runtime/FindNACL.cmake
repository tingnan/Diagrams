# Cross-compiling requires CMake 2.6 or newer. To cross-compile, first modify
# this file to set the proper settings and paths. Then use it from build/ like:
# cmake .. -DCMAKE_TOOLCHAIN_FILE=../XCompile.txt \
#          -DCMAKE_INSTALL_PREFIX=/usr/mingw32/mingw
# If you already have a toolchain file setup, you may use that instead of this
# file.

SET(NACL 1)

# find the SDK root
IF("$ENV{NACL_SDK_ROOT}" STREQUAL "")
  MESSAGE(STATUS "NACL_SDK_ROOT not found! Set NACL_PATH to HOME/nacl_sdk/pepper_canary")
  SET(NACL_SDK_ROOT "$ENV{HOME}/nacl_sdk/pepper_canary")
ELSE()
  SET(NACL_SDK_ROOT "$ENV{NACL_SDK_ROOT}")
ENDIF()

# set system architecture
# can be x86_64 i686 arm or pnacl
# SET(NACL_ARCH ${CMAKE_HOST_SYSTEM_PROCESSOR})
# the name of the target operating system

SET(NACL_ARCH "pnacl")
SET(CMAKE_SYSTEM_NAME Linux)

# get the toolchain, in order to use c++11 we have to use linux_pnacl toolchain
SET(NACL_TOOLCHAIN_ROOT "${NACL_SDK_ROOT}/toolchain/linux_pnacl")
# set the prefix for compiler tools
IF(${NACL_ARCH} STREQUAL "pnacl")
  SET(NACL_CROSS_PREFIX "pnacl")
ELSE()
  SET(NACL_CROSS_PREFIX "${NACL_ARCH}-nacl")
ENDIF()

# which compilers to use for C and C++
SET(CMAKE_AR "${NACL_TOOLCHAIN_ROOT}/bin/${NACL_CROSS_PREFIX}-ar" CACHE FILEPATH "Archiver")
SET(CMAKE_LINKER "${NACL_TOOLCHAIN_ROOT}/bin/${NACL_CROSS_PREFIX}-ld" CACHE FILEPATH "Linker")
SET(CMAKE_RANLIB "${NACL_TOOLCHAIN_ROOT}/bin/${NACL_CROSS_PREFIX}-ranlib" CACHE FILEPATH "Ranlib")
SET(CMAKE_C_COMPILER "${NACL_TOOLCHAIN_ROOT}/bin/${NACL_CROSS_PREFIX}-clang")
SET(CMAKE_CXX_COMPILER "${NACL_TOOLCHAIN_ROOT}/bin/${NACL_CROSS_PREFIX}-clang++")

IF(${NACL_ARCH} STREQUAL "pnacl")
  SET(NACL_SDK_LIBDIR "${NACL_SDK_ROOT}/lib/${NACL_ARCH}/Release")
  SET(CMAKE_FIND_ROOT_PATH "${NACL_SDK_LIBDIR}" "${NACL_TOOLCHAIN_ROOT}/le32-nacl")
ELSE()
  SET(NACL_SDK_LIBDIR "${NACL_SDK_ROOT}/lib/clang-newlib_${NACL_ARCH}/Release")
  SET(CMAKE_FIND_ROOT_PATH "${NACL_SDK_LIBDIR}" "${NACL_TOOLCHAIN_ROOT}/${NACL_CROSS_PREFIX}")
ENDIF()



# Reference CMAKE_TOOLCHAIN_FILE and CMAKE_PREFIX_PATH here to avoid
# cmake warnings:
# http://public.kitware.com/pipermail/cmake/2013-February/053476.html
SET(DUMMY_VALUE ${CMAKE_TOOLCHAIN_FILE} ${CMAKE_PREFIX_PATH})


# adjust the default behaviour of the FIND_XXX() commands:
# search headers and libraries in the target environment, search 
# programs in the host environment
SET(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
SET(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
SET(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
# nacl abi says 32bits little endian
SET(CMAKE_SIZEOF_VOID_P 4)