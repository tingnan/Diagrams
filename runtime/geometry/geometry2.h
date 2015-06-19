// the geometry

#ifndef _DIAGRAMMAR_GEOMETRY2D_
#define _DIAGRAMMAR_GEOMETRY2D_
#include <vector>
#include "typedefs.h"

namespace diagrammar {

// we may eventually add primitives to facilitate fast computation

struct Triangle2D {
  Vec2f p0;
  Vec2f p1;
  Vec2f p2;
};

class Geometry2D {
 public:
 private:
  std::vector<Vec2f> path_;
  bool is_closed_ = true;
  std::vector<std::vector<Vec2f> > holes_;
  std::vector<Vec2f> _Simplify(const std::vector<Vec2f>&);
 public:
  Geometry2D() = default;
  Geometry2D(const std::vector<Vec2f>& pts);
  Geometry2D(const Geometry2D&) = default;
  Geometry2D(Geometry2D&&) = default;
  void SetPath(const std::vector<Vec2f>& pts);
  void SetHole(int i, const std::vector<Vec2f>& pts);
  void AddHole(const std::vector<Vec2f>& pts);
  size_t GetNumHoles() { return holes_.size(); }
  std::vector<Triangle2D> Triangulate() const;
  const std::vector<Vec2f>& GetPath() const;
  const std::vector<Vec2f>& GetHole(int i) const;
  const std::vector<std::vector<Vec2f> >& GetHoles() const { return holes_; }
  void SetPathClosed(bool flag) { is_closed_ = flag; }
  bool IsPathClosed() const { return is_closed_; }
};
}

#endif
