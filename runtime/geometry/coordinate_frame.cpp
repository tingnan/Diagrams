// Copyright 2015 Native Client Authors.

#include <iostream>

#include "geometry/coordinate_frame.h"

namespace diagrammar {
CoordinateFrame::CoordinateFrame() { frame_ = Isometry3f::Identity(); }

CoordinateFrame::CoordinateFrame(const Isometry2f& transform) {
  frame_.linear().topLeftCorner<2, 2>() = transform.linear();
  frame_.translation() =
      Vector3f(transform.translation()(0), transform.translation()(1), 0);
}

// notice: it differs from SetTranslation because
// it adds on top of existing translation
void CoordinateFrame::Translate(const Vector3f& disp) {
  // cannot use mTransform.translate() because the order is going to be WRONG
  // our isometry is composed in the order Translation() * Rotation()
  // so only pretranslate will work correctly!
  frame_.pretranslate(disp);
}

void CoordinateFrame::SetTranslation(const Vector3f& disp) {
  frame_.translation() = disp;
}

// Similarly, we have SetRotation and Rotate function the rotate functions here
// ONLY rotate the frame about its own center (NOT about the parent's center).
// i.e. the rotation is IN PLACE.
void CoordinateFrame::Rotate(const Matrix3f& rot_mat) {
  frame_.linear() = rot_mat * frame_.rotation();
}
void CoordinateFrame::Rotate(const AngleAxisf& rotation) {
  frame_.linear() = rotation * frame_.rotation();
}
void CoordinateFrame::SetRotation(const Matrix3f& rot_mat) {
  frame_.linear() = rot_mat;
}
void CoordinateFrame::SetRotation(const AngleAxisf& rotation) {
  frame_.linear() = rotation.matrix();
}

// utility methods acting on vectors according to the coordinate transformation
Vector3f CoordinateFrame::TransformVector(const Vector3f& vec) const {
  return frame_.linear() * vec;
}

Vector3f CoordinateFrame::InverseTransformVector(const Vector3f& vec) const {
  return frame_.inverse().linear() * vec;
}

// give the coordinate in the parent frame
Vector3f CoordinateFrame::TransformPoint(const Vector3f& vec) const {
  return frame_ * vec;
}
// transform the coordinate from the parent to local
Vector3f CoordinateFrame::InverseTransformPoint(const Vector3f& vec) const {
  return frame_.inverse() * vec;
}

Vector3f CoordinateFrame::GetTranslation() const {
  return frame_.translation();
}

Quaternionf CoordinateFrame::GetRotation() const {
  return Quaternionf(frame_.rotation());
}

Matrix3f CoordinateFrame::GetRotationMatrix() const { return frame_.linear(); }

}  // namespace diagrammar
