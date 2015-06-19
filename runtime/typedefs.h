#ifndef _DIAGRAMMAR_
#define _DIAGRAMMAR_
#include <Eigen/Core> 

namespace diagrammar {
// Eigen is a heavy template library
// Eigen::Vector2f and Matrix2f are all typedefs
// we have to at least include the Core module
typedef Eigen::Vector2f Vec2f;
typedef Eigen::Matrix2f Mat2f;
}

#endif