// Copyright 2015 Native Client authors
#ifndef RUNTIME_GEOMETRY_GEOMETRY2_H_
#define RUNTIME_GEOMETRY_GEOMETRY2_H_
#include <vector>
#include "typedefs.h"

namespace diagrammar {

// we may eventually add primitives to facilitate fast computation

struct Triangle2D {
  Vec2f p0;
  Vec2f p1;
  Vec2f p2;
};

class ComplexShape2D {
 public:
  ComplexShape2D() = default;
  explicit ComplexShape2D(const std::vector<Vec2f>& pts);
  ComplexShape2D(const ComplexShape2D&) = default;
  ComplexShape2D(ComplexShape2D&&) = default;
  ComplexShape2D& operator = (const ComplexShape2D&) = default;
  ComplexShape2D& operator = (ComplexShape2D&&) = default;
  void SetPath(const std::vector<Vec2f>& pts);
  void SetHole(int i, const std::vector<Vec2f>& pts);
  void AddHole(const std::vector<Vec2f>& pts);
  size_t GetNumHoles() { return holes_.size(); }
  std::vector<Triangle2D> Triangulate() const;
  const std::vector<Vec2f>& GetPath() const;
  const std::vector<Vec2f>& GetHole(int i) const;
  const std::vector<std::vector<Vec2f> >& GetHoles() const;
  void SetPathClosed(bool flag);
  bool IsPathClosed() const { return is_closed_; }
 private:
  std::vector<Vec2f> path_;
  bool is_closed_ = true;
  std::vector<std::vector<Vec2f> > holes_;
  std::vector<Vec2f> _Simplify(const std::vector<Vec2f>&);
};
}  // namespace diagrammar

#endif  // RUNTIME_GEOMETRY_GEOMETRY2_H_
