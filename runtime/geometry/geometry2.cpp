#include "geometry2.h"
#include "triangulate.h"
#include <iostream>
namespace diagrammar {

ComplexShape2D::ComplexShape2D(const std::vector<Vec2f>& pts) {
  path_ = _Simplify(pts);
}

void ComplexShape2D::SetPath(const std::vector<Vec2f>& pts) {
  path_ = _Simplify(pts);
}

void ComplexShape2D::SetHole(int i, const std::vector<Vec2f>& pts) {
  assert(is_closed_);
  holes_[i] = _Simplify(pts);
}

void ComplexShape2D::AddHole(const std::vector<Vec2f>& pts) {
  assert(is_closed_);
  holes_.emplace_back(_Simplify(pts));
}

std::vector<Triangle2D> ComplexShape2D::Triangulate() const {
  if (is_closed_) return DelaunayTriangulation(path_, holes_);

  return InflateAndTriangulate(path_);
}

inline std::vector<Vec2f> ComplexShape2D::_Simplify(const std::vector<Vec2f>& in) {
  return Simplify(in);
}

const std::vector<Vec2f>& ComplexShape2D::GetPath() const { return path_; }

const std::vector<Vec2f>& ComplexShape2D::GetHole(int i) const {
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
  // we also want to send out notifications to the drawer
  // TO DO
}

}