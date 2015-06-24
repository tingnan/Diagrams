// Copyright 2015 Native Client Authors.
#include <iostream>
#include <vector>
#include "geometry/geometry2.h"
#include "geometry/triangulate.h"

namespace diagrammar {

ComplexShape2D::ComplexShape2D(const std::vector<Vector2f>& pts) {
  path_ = SimplifyPath(pts);
}

void ComplexShape2D::SetPath(const std::vector<Vector2f>& pts) {
  path_ = SimplifyPath(pts);
}

void ComplexShape2D::SetHole(int i, const std::vector<Vector2f>& pts) {
  assert(is_closed_);
  holes_[i] = SimplifyPath(pts);
}

void ComplexShape2D::AddHole(const std::vector<Vector2f>& pts) {
  assert(is_closed_);
  holes_.emplace_back(std::move(SimplifyPath(pts)));
}

std::vector<Triangle2D> ComplexShape2D::Triangulate() const {
  if (is_closed_) return DelaunayTriangulation(path_, holes_);

  return InflateAndTriangulate(path_);
}

inline std::vector<Vector2f> ComplexShape2D::SimplifyPath(
    const std::vector<Vector2f>& in) {
  return Simplify(in);
}

std::vector<Vector2f> ComplexShape2D::GetPath() const { return path_; }

std::vector<Vector2f> ComplexShape2D::GetHole(int i) const {
  assert(i >= 0);
  assert(i < holes_.size());
  return holes_[i];
}

void ComplexShape2D::SetPathClosed(bool flag) {
  // if now we want to open the path
  if (!flag) {
    holes_.clear();
  }
  is_closed_ = flag;
}

}  // namespace diagrammar
