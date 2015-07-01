// Copyright 2015 Native Client Authors.

#include <iostream>

#include "geometry/geometry2.h"
#include "geometry/triangulate.h"

namespace diagrammar {

ComplexPolygon::ComplexPolygon(const std::vector<Vector2f>& pts) {
  path_ = SimplifyPolyline(pts);
}

void ComplexPolygon::SetPath(const std::vector<Vector2f>& pts) {
  path_ = SimplifyPolyline(pts);
}

void ComplexPolygon::SetHole(int i, const std::vector<Vector2f>& pts) {
  assert(is_closed_);
  holes_[i] = SimplifyPolyline(pts);
}

void ComplexPolygon::AddHole(const std::vector<Vector2f>& pts) {
  assert(is_closed_);
  holes_.emplace_back(std::move(SimplifyPolyline(pts)));
}

std::vector<Triangle2D> ComplexPolygon::Triangulate() const {
  if (is_closed_) return TriangulatePolygon(path_, holes_);

  return TriangulatePolyline(path_);
}

std::vector<Vector2f> ComplexPolygon::GetPath() const { return path_; }

std::vector<Vector2f> ComplexPolygon::GetHole(int i) const {
  assert(i >= 0);
  assert(i < holes_.size());
  return holes_[i];
}

void ComplexPolygon::SetPathClosed(bool closed) {
  // if now we want to open the path
  if (!closed) {
    holes_.clear();
  }
  is_closed_ = closed;
}

}  // namespace diagrammar
