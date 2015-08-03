// Copyright 2015 Native Client Authors.

#ifndef RUNTIME_GEOMETRY_GEOMETRY3_H_
#define RUNTIME_GEOMETRY_GEOMETRY3_H_

#include <array>
#include <vector>

#include "include/matrix_types.h"
#include "geometry/coordinate_frame.h"
#include "geometry/geometry2.h"

namespace diagrammar {

struct TriangleMesh3D {
  std::vector<Vector3f> vertices;
  // Each face stores 3 indices to the actual vertex in the "vertices" vector
  // Warning: clang-newlib cannot handle std::array correctly right now
  std::vector<std::array<size_t, 3> > faces;
};

enum class Shape3DType { kPlane, kSphere, kConvexHull };

// base class for the polymorphic type
struct CollisionShape3D {
 public:
  virtual ~CollisionShape() = 0;
};

struct CollisionShapePlaneShape : public CollisionShape3D {
 private:
  CoordinateFrame frame_;
  // The actual shape lies in x-y plane in its local coordinate fram
  std::unique_ptr<CollisionShape2D> shape_;
};

}  // namespace diagrammar

#endif  // RUNTIME_GEOMETRY_GEOMETRY2_H_
