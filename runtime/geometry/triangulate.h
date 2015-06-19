#ifndef _DIAGRAMMAR_TRIANGULATION_
#define _DIAGRAMMAR_TRIANGULATION_
#include <vector>
#include "typedefs.h"

namespace diagrammar {
struct Triangle2D;
class Triangulation {
 public:
  enum PolylineMethod { kDouglasPeucker };

 private:
  // recursive method, the output vector indicates whether we would want to keep
  // a point
  void _PolylineDouglasPeuckerRecursive(const size_t bg, const size_t ed,
                                        const std::vector<Vec2f>&, float tol,
                                        std::vector<bool>&);
  // the iterative method
  // however DouglasPeucker method does not check for intersections!
  // in some unlucky case, it changes the topology of the curve
  std::vector<Vec2f> _PolylineDouglasPeuckerIterative(const std::vector<Vec2f>&,
                                                      float tol);

  // EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // Eigen::Vector4f is a fixed-size vectorizable Eigen object
  std::vector<Triangle2D> _DelaunaySweepline(
      const std::vector<Vec2f>& path,
      const std::vector<std::vector<Vec2f> >* holes);

 public:
  Triangulation();
  std::vector<Vec2f> Simplify(
      const std::vector<Vec2f>& in,
      PolylineMethod m = PolylineMethod::kDouglasPeucker);
  std::vector<Triangle2D> DelaunayTriangulation(const std::vector<Vec2f>& path);
  std::vector<Triangle2D> DelaunayTriangulation(
      const std::vector<Vec2f>& path,
      const std::vector<std::vector<Vec2f> >& holes);
  std::vector<Triangle2D> InflateAndTriangulate(const std::vector<Vec2f>& path);
};
}

#endif