#include "coordinate_frame.h"

namespace diagrammar {
CoordinateFrame2D::CoordinateFrame2D() {
  frame_ = Eigen::Isometry2f::Identity();
}

CoordinateFrame2D::CoordinateFrame2D(const Eigen::Isometry2f& transform)
    : frame_(transform) {}

// notice: it differs from SetTranslation because
// it adds on top of existing translation
void CoordinateFrame2D::Translate(const Vec2f& disp) {
  // cannot use mTransform.translate() because the order is going to be WRONG
  // our isometry is composed in the order Translation() * Rotation()
  // so only pretranslate will work correctly!
  frame_.pretranslate(disp);
}

void CoordinateFrame2D::SetTranslation(const Vec2f& disp) {
  frame_.translation() = disp;
}

// similarly, we have SetRotation and Rotate function
// the rotate functions here ONLY rotate the frame
// about its center. It is NOT about the parent's center
// i.e. the rotation is IN PLACE.
void CoordinateFrame2D::Rotate(const Mat2f& rot_mat) {
  // careful! rotations does not commute in general
  frame_.linear() = rot_mat * frame_.rotation();
}

void CoordinateFrame2D::Rotate(const Eigen::Rotation2Df& rot) {
  // careful! rotations does not commute in general
  frame_.linear() = rot * frame_.rotation();
}

void CoordinateFrame2D::SetRotation(const Mat2f& rot_mat) {
  frame_.linear() = rot_mat;
}

void CoordinateFrame2D::SetRotation(const Eigen::Rotation2Df& rot) {
  frame_.linear() = rot.matrix();
}

// utility methods acting on vectors according to the coordinate transformation
Vec2f CoordinateFrame2D::TransformVector(const Vec2f& vec) const {
  return frame_.linear() * vec;
}

Vec2f CoordinateFrame2D::InverseTransformVector(const Vec2f& vec) const {
  return frame_.inverse().linear() * vec;
}

// give the coordinate in the parent frame
Vec2f CoordinateFrame2D::TransformPoint(const Vec2f& vec) const {
  return frame_ * vec;
}
// transform the coordinate from the parent to local
Vec2f CoordinateFrame2D::InverseTransformPoint(const Vec2f& vec) const {
  return frame_.inverse() * vec;
}

Vec2f CoordinateFrame2D::GetTranslation() { return frame_.translation(); }

Eigen::Rotation2Df CoordinateFrame2D::GetRotation() {
  Eigen::Rotation2Df tmp = Eigen::Rotation2Df::Identity();
  tmp.fromRotationMatrix(frame_.linear());
  return tmp;
}
}