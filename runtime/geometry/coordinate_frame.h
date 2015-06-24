// Copyright 2015 Native Client Authors.
#ifndef RUNTIME_GEOMETRY_COORDINATE_FRAME_H_
#define RUNTIME_GEOMETRY_COORDINATE_FRAME_H_
#include "include/typedefs.h"

namespace diagrammar {
// a frame that defines the transformation of the object
// notice we construct the isometry in the order:
// Translation * Rotation
// so any vector is first Rotated and then translated!
// a vector v can be transformed in to the parent frame using:
// Translation * Rotation * v
// This CoordinateFrame part needs precaution when modifying
class CoordinateFrame2D {
 public:
  CoordinateFrame2D();
  explicit CoordinateFrame2D(const Isometry2f& transform);
  CoordinateFrame2D(const CoordinateFrame2D&) = default;
  CoordinateFrame2D(CoordinateFrame2D&&) = default;
  CoordinateFrame2D& operator=(const CoordinateFrame2D&) = default;
  CoordinateFrame2D& operator=(CoordinateFrame2D&&) = default;

 public:
  // notice: it differs from SetTranslation because
  // it adds on top of existing translation
  void Translate(const Vector2f& disp);

  void SetTranslation(const Vector2f& disp);

  // similarly, we have SetRotation and Rotate function
  // the rotate functions here ONLY rotate the frame
  // about its center. It is NOT about the parent's center
  // i.e. the rotation is IN PLACE.
  void Rotate(const Matrix2f& rotmat);

  void Rotate(const Rotation2f& rot);

  void SetRotation(const Matrix2f& rotmat);

  void SetRotation(const Rotation2f& rot);

  void RotateAboutParentCenter(const Matrix2f& rotmat);
  void RotateAboutParentCenter(const Rotation2f& rot);
  // utility methods acting on vectors according to the coordinate
  // transformation
  Vector2f TransformVector(const Vector2f& vec) const;

  Vector2f InverseTransformVector(const Vector2f& vec) const;

  // give the coordinate in the parent frame
  Vector2f TransformPoint(const Vector2f& vec) const;
  // transform the coordinate from the parent to local
  Vector2f InverseTransformPoint(const Vector2f& vec) const;

  Vector2f GetTranslation();
  Rotation2f GetRotation();

 private:
  // our coordinate frame can be treated as a isometry
  Isometry2f frame_;
};
}  // namespace diagrammar

#endif  // RUNTIME_GEOMETRY_COORDINATE_FRAME_H_
