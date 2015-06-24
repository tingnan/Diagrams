#ifndef _DIAGRAMMAR_
#define _DIAGRAMMAR_
#include <Eigen/Core> 
#include <Eigen/Geometry>
namespace diagrammar {
// Eigen is a heavy template library
// Eigen::Vector2f and Matrix2f are all typedefs
// we have to at least include the Core module
typedef ::Eigen::Vector2f Vector2f;
typedef ::Eigen::Matrix2f Matrix2f;
typedef ::Eigen::Rotation2Df Rotation2f;
typedef ::Eigen::Isometry2f Isometry2f;
}

#endif