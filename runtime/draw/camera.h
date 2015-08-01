// Copy right 2015 Native Client Authors

#ifndef RUNTIME_DRAW_CAMERA_H_
#define RUNTIME_DRAW_CAMERA_H_

#include "include/matrix_types.h"

namespace diagrammar {

class Camera {
 public:
  Camera() = default;
  // Set up the camera position and the frustum
  Camera(const Vector3f &position, const Vector3f &target, const Vector3f &up,
         float fov = M_PI / 2.0, float aspect_ratio = 1, float near = 10.0,
         float far = 10000.0);
  void SetView(const Vector3f &position, const Vector3f &target,
               const Vector3f &up);
  void SetPerspective(float fov, float aspect_ratio, float near, float far);
  void Translate(const Vector3f &translation);
  const Matrix4f &GetViewProjection();
  // Matrix4f member needs this alignment.
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 private:
  // The camera position and frame orientation is defined by a isometry3f.
  // The camera will look at -z position.
  // the camera modelview matrix is kept separately (for later api/usage).
  Matrix4f view_ = Matrix4f::Identity();
  Matrix4f perspective_projection_ = Matrix4f::Identity();
  bool matrix_dirty_ = false;
  Matrix4f cached_view_projection_ = perspective_projection_ * view_;
};

}  // namespace diagrammar

#endif  // RUNTIME_DRAW_CAMERA_H_