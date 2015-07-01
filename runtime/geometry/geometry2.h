// Copyright 2015 Native Client Authors.

#ifndef RUNTIME_GEOMETRY_GEOMETRY2_H_
#define RUNTIME_GEOMETRY_GEOMETRY2_H_

#include <vector>
#include "include/matrix_types.h"

namespace diagrammar {

// we may eventually add primitives to facilitate fast computation

struct Triangle2D {
  Vector2f p0;
  Vector2f p1;
  Vector2f p2;
};


// This is the glue code, for some physics engines we need to triangulate
// the polylines. Other physics engine can use convex hull decomposition
class ComplexPolygon {
 public:
  ComplexPolygon() = default;
  // construct a shape with boundary described by input points
  explicit ComplexPolygon(const std::vector<Vector2f>& pts);
  ComplexPolygon(const ComplexPolygon&) = default;
  ComplexPolygon(ComplexPolygon&&) = default;
  
  ComplexPolygon& operator=(const ComplexPolygon&) = default;
  ComplexPolygon& operator=(ComplexPolygon&&) = default;

  std::vector<Vector2f> GetPath() const;
  void SetPath(const std::vector<Vector2f>& pts);
  
  size_t GetNumHoles() { return holes_.size(); }
  void SetHole(int i, const std::vector<Vector2f>& pts);
  void AddHole(const std::vector<Vector2f>& pts);

  // return a triangulation of the shape
  std::vector<Triangle2D> Triangulate() const;
  // get the hole at index i
  std::vector<Vector2f> GetHole(int i) const;
  // change the path type (open or close)
  void SetPathClosed(bool flag);
  bool IsPathClosed() const { return is_closed_; }

 private:

  std::vector<Vector2f> path_;
  bool is_closed_ = true;
  std::vector<std::vector<Vector2f> > holes_;
};

}  // namespace diagrammar

#endif  // RUNTIME_GEOMETRY_GEOMETRY2_H_
