// Copyright 2015 Native Client Authors.
#ifndef RUNTIME_GEOMETRY_GEOMETRY2_H_
#define RUNTIME_GEOMETRY_GEOMETRY2_H_
#include <vector>
#include "include/typedefs.h"

namespace diagrammar {

// we may eventually add primitives to facilitate fast computation

struct Triangle2D {
  Vector2f p0;
  Vector2f p1;
  Vector2f p2;
};

class ComplexShape2D {
 public:
  // construct a empty shape
  ComplexShape2D() = default;
  // construct a shape with boundary described by input points
  explicit ComplexShape2D(const std::vector<Vector2f>& pts);
  // all defaut
  ComplexShape2D(const ComplexShape2D&) = default;
  ComplexShape2D(ComplexShape2D&&) = default;
  ComplexShape2D& operator=(const ComplexShape2D&) = default;
  ComplexShape2D& operator=(ComplexShape2D&&) = default;
  // change the boundary path
  void SetPath(const std::vector<Vector2f>& pts);
  // change the hole at index i
  void SetHole(int i, const std::vector<Vector2f>& pts);
  // add a hole to the shape
  void AddHole(const std::vector<Vector2f>& pts);
  size_t GetNumHoles() { return holes_.size(); }
  // return a triangulation of the shape
  std::vector<Triangle2D> Triangulate() const;
  // get the boundary points
  std::vector<Vector2f> GetPath() const;
  // get the hole at index i
  std::vector<Vector2f> GetHole(int i) const;
  // change the path type (open or close)
  void SetPathClosed(bool flag);
  bool IsPathClosed() const { return is_closed_; }

 private:
  std::vector<Vector2f> path_;
  bool is_closed_ = true;
  std::vector<std::vector<Vector2f> > holes_;
  std::vector<Vector2f> SimplifyPath(const std::vector<Vector2f>&);
};

}  // namespace diagrammar

#endif  // RUNTIME_GEOMETRY_GEOMETRY2_H_
