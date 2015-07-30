// Copyright 2015 Native Client Authors.

#ifndef RUNTIME_GEOMETRY_COORDINATE_FRAME_H_
#define RUNTIME_GEOMETRY_COORDINATE_FRAME_H_

#include "include/matrix_types.h"

namespace diagrammar {
// a frame that defines the transformation of the object
// notice we construct the isometry in the order:
// Translation * Rotation
// so any vector is first Rotated and then translated!
// a vector v can be transformed in to the parent frame using:
// Translation * Rotation * v
// This CoordinateFrame part needs precaution when modifying
class CoordinateFrame {
 public:
  CoordinateFrame();
  explicit CoordinateFrame(const Isometry2f& transform);
  explicit CoordinateFrame(const Isometry3f& transform) : frame_(transform) {}

  void Translate(const Vector3f& disp);
  void SetTranslation(const Vector3f& disp);

  void Rotate(const Matrix3f& rot_mat);
  void Rotate(const AngleAxisf& rotation);
  void SetRotation(const Matrix3f& rot_mat);
  void SetRotation(const AngleAxisf& rotation);

  // void RotateAboutParentCenter(const Matrix2f& rotmat);
  // void RotateAboutParentCenter(const Rotation2f& rot);

  Vector3f TransformVector(const Vector3f& vec) const;
  Vector3f InverseTransformVector(const Vector3f& vec) const;
  Vector3f TransformPoint(const Vector3f& vec) const;
  Vector3f InverseTransformPoint(const Vector3f& vec) const;

  // Get the translation of the frame (relative to the parent frame).
  Vector3f GetTranslation() const;
  // The most common representation of rotation
  Quaternionf GetRotation() const;
  Matrix3f GetRotationMatrix() const;
  // Alignment required for Isometry3f member
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 private:
  // our coordinate frame can be treated as a isometry
  // 3x3 matrix
  Isometry3f frame_ = Isometry3f::Identity();
};
}  // namespace diagrammar

#endif  // RUNTIME_GEOMETRY_COORDINATE_FRAME_H_
