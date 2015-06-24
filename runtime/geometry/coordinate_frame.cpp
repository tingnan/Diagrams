// Copyright 2015 Native Client Authors.
#include "geometry/coordinate_frame.h"

namespace diagrammar {
CoordinateFrame2D::CoordinateFrame2D() { frame_ = Isometry2f::Identity(); }

CoordinateFrame2D::CoordinateFrame2D(const Isometry2f& transform)
    : frame_(transform) {}

// notice: it differs from SetTranslation because
// it adds on top of existing translation
void CoordinateFrame2D::Translate(const Vector2f& disp) {
  // cannot use mTransform.translate() because the order is going to be WRONG
  // our isometry is composed in the order Translation() * Rotation()
  // so only pretranslate will work correctly!
  frame_.pretranslate(disp);
}

void CoordinateFrame2D::SetTranslation(const Vector2f& disp) {
  frame_.translation() = disp;
}

// similarly, we have SetRotation and Rotate function
// the rotate functions here ONLY rotate the frame
// about its center. It is NOT about the parent's center
// i.e. the rotation is IN PLACE.
void CoordinateFrame2D::Rotate(const Matrix2f& rot_mat) {
  // careful! rotations does not commute in general
  frame_.linear() = rot_mat * frame_.rotation();
}

void CoordinateFrame2D::Rotate(const Rotation2f& rot) {
  // careful! rotations does not commute in general
  frame_.linear() = rot * frame_.rotation();
}

void CoordinateFrame2D::SetRotation(const Matrix2f& rot_mat) {
  frame_.linear() = rot_mat;
}

void CoordinateFrame2D::SetRotation(const Rotation2f& rot) {
  frame_.linear() = rot.matrix();
}

// utility methods acting on vectors according to the coordinate transformation
Vector2f CoordinateFrame2D::TransformVector(const Vector2f& vec) const {
  return frame_.linear() * vec;
}

Vector2f CoordinateFrame2D::InverseTransformVector(const Vector2f& vec) const {
  return frame_.inverse().linear() * vec;
}

// give the coordinate in the parent frame
Vector2f CoordinateFrame2D::TransformPoint(const Vector2f& vec) const {
  return frame_ * vec;
}
// transform the coordinate from the parent to local
Vector2f CoordinateFrame2D::InverseTransformPoint(const Vector2f& vec) const {
  return frame_.inverse() * vec;
}

Vector2f CoordinateFrame2D::GetTranslation() const {
  return frame_.translation();
}

float CoordinateFrame2D::GetRotationAngle() const {
  Rotation2f tmp = Rotation2f::Identity();
  tmp.fromRotationMatrix(frame_.linear());
  return tmp.angle();
}

Matrix2f CoordinateFrame2D::GetRotationMatrix() const {
  return frame_.linear();
}

}  // namespace diagrammar
