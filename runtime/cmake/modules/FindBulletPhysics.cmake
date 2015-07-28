find_path(BULLETPHYSICS_INCLUDE_DIRS NAMES btBulletCollisionCommon.h HINTS PATH_SUFFIXES include/bullet/)
find_library(BULLETDYNAMIC_LIBRARIES NAMES BulletDynamics)
find_library(BULLETCOLLISION_LIBRARIES NAMES BulletCollision)
find_library(BULLETLINEARMATH_LIBRARIES NAMES LinearMath)
set(BULLETPHYSICS_LIBRARIES ${BULLETDYNAMIC_LIBRARIES} ${BULLETCOLLISION_LIBRARIES} ${BULLETLINEARMATH_LIBRARIES})
