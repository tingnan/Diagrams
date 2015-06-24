// Copyright 2015 Native Client Authors
#ifndef RUNTIME_INCLUDE_H_
#define RUNTIME_INCLUDE_H_
#include <Eigen/Core>
#include <Eigen/Geometry>
namespace diagrammar {
typedef ::Eigen::Vector2f Vector2f;
typedef ::Eigen::Vector3f Vector3f;
typedef ::Eigen::Matrix2f Matrix2f;
typedef ::Eigen::Matrix3f Matrix3f;
typedef ::Eigen::Matrix4f Matrix4f;
typedef ::Eigen::Isometry2f Isometry2f;
typedef ::Eigen::Isometry3f Isometry3f;
typedef ::Eigen::Rotation2Df Rotation2f;
}  // namespace diagrammar

#endif  // RUNTIME_INCLUDE_H_