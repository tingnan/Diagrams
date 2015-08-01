// Copyright 2015 Native Client Authors

#include <cmath>

#include <iostream>

#include <draw/camera.h>

namespace diagrammar {

Camera::Camera(const Vector3f &position, const Vector3f &target,
               const Vector3f &up, float fov, float aspect_ratio, float near,
               float far)
    : view_(Matrix4f::Identity()),
      perspective_projection_(Matrix4f::Identity()) {
  SetView(position, target, up);
  SetPerspective(fov, aspect_ratio, near, far);
}

void Camera::SetView(const Vector3f &position, const Vector3f &target,
                     const Vector3f &up) {
  Matrix3f rotation;
  rotation.row(1) = up.normalized();
  rotation.row(2) = (position - target).normalized();
  rotation.row(0) = rotation.row(1).cross(rotation.row(2));
  view_.topLeftCorner<3, 3>() = rotation;
  view_.topRightCorner<3, 1>() = -rotation * position;
  view_(3, 3) = 1.0;
  matrix_dirty_ = true;
}

void Camera::SetPerspective(float fov, float aspect_ratio, float near,
                            float far) {
  float theta = fov * 0.5;
  float range = far - near;
  float inv_tan = 1. / tan(theta);
  perspective_projection_(0, 0) = aspect_ratio * inv_tan;
  perspective_projection_(1, 1) = inv_tan;
  perspective_projection_(2, 2) = -(far + near) / range;
  perspective_projection_(3, 2) = -1;
  perspective_projection_(2, 3) = -2 * far * near / range;
  perspective_projection_(3, 3) = 0;
  matrix_dirty_ = true;
}

void Camera::Translate(const Vector3f &translation) {
  // Compute the frame in the world coordinate;
  Matrix4f frame = view_.inverse();
  frame.topRightCorner<3, 1>() += translation;
  view_ = frame.inverse();
  matrix_dirty_ = true;
}

const Matrix4f &Camera::GetViewProjection() {
  if (matrix_dirty_) {
    matrix_dirty_ = false;
    cached_view_projection_ = perspective_projection_ * view_;
  }

  return cached_view_projection_;
}

} // namespace diagrammar
