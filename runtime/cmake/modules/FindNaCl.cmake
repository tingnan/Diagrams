find_library(PPAPI_CPP_LIBRARIES NAMES ppapi_cpp PATHS ${NACL_SDK_LIBDIR})
find_library(PPAPI_LIBRARIES NAMES ppapi PATHS ${NACL_SDK_LIBDIR})
find_library(PPAPI_SIMPLE_LIBRARIES NAMES ppapi_simple PATHS ${NACL_SDK_LIBDIR})
find_library(PPAPI_SIMPLE_CPP_LIBRARIES NAMES ppapi_simple_cpp PATHS ${NACL_SDK_LIBDIR})
find_library(PPAPI_GLES2_LIBRARIES NAMES ppapi_gles2 PATHS ${NACL_SDK_LIBDIR})
find_library(NACL_IO_LIBRARIES NAMES nacl_io PATHS ${NACL_SDK_LIBDIR})

set(NACL_LIBRARIES ${PPAPI_GLES2_LIBRARIES} ${PPAPI_SIMPLE_LIBRARIES} ${PPAPI_LIBRARIES} ${NACL_IO_LIBRARIES} )
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(NACL DEFAULT_MSG
                                  NACL_LIBRARIES)
mark_as_advanced(NACL_LIBRARIES)
