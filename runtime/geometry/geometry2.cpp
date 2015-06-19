#include "geometry2.h"
#include "triangulate.h"
#include <iostream>
namespace diagrammar {

Geometry2D::Geometry2D(const std::vector<Vec2f>& pts) {
  path_ = _Simplify(pts);
}

void Geometry2D::SetPath(const std::vector<Vec2f>& pts) {
  path_ = _Simplify(pts);
}

void Geometry2D::SetHole(int i, const std::vector<Vec2f>& pts) {
  assert(is_closed_);
  holes_[i] = _Simplify(pts);
}

void Geometry2D::AddHole(const std::vector<Vec2f>& pts) {
  assert(is_closed_);
  holes_.emplace_back(_Simplify(pts));
}

std::vector<Triangle2D> Geometry2D::Triangulate() const {
  Triangulation worker;
  if (is_closed_) return worker.DelaunayTriangulation(path_, holes_);

  return worker.InflateAndTriangulate(path_);
}

inline std::vector<Vec2f> Geometry2D::_Simplify(const std::vector<Vec2f>& in) {
  Triangulation worker;
  return worker.Simplify(in);
}

const std::vector<Vec2f>& Geometry2D::GetPath() const { return path_; }

const std::vector<Vec2f>& Geometry2D::GetHole(int i) const {
  return holes_[i];
}
}