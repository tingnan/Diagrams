# will use the nacl_sdk version of jsoncpp, instead of the naclports version
find_path(JSONCPP_INCLUDE_DIRS NAMES json/json.h HINTS PATH_SUFFIXES jsoncpp)
find_library(JSONCPP_LIBRARIES NAMES jsoncpp PATHS ${NACL_SDK_LIBDIR})
