find_path(EIGEN3_INCLUDE_DIRS NAMES Eigen/Core HINTS PATH_SUFFIXES include/eigen3/)
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(EIGEN3 DEFAULT_MSG
                                  EIGEN3_INCLUDE_DIRS)
mark_as_advanced(EIGEN3_INCLUDE_DIRS)
