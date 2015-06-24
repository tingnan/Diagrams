// Copyright 2015 Native Client Authors.
#ifndef RUNTIME_GEOMETRY_TRIANGULATE_H_
#define RUNTIME_GEOMETRY_TRIANGULATE_H_
#include <vector>
#include "include/typedefs.h"

namespace diagrammar {
struct Triangle2D;
enum PolylineMethod { kDouglasPeucker };
std::vector<Vector2f> Simplify(
    const std::vector<Vector2f>& in,
    PolylineMethod m = PolylineMethod::kDouglasPeucker);
std::vector<Triangle2D> DelaunayTriangulation(const std::vector<Vector2f>& path);
std::vector<Triangle2D> DelaunayTriangulation(
    const std::vector<Vector2f>& path,
    const std::vector<std::vector<Vector2f> >& holes);
std::vector<Triangle2D> InflateAndTriangulate(const std::vector<Vector2f>& path);
}  // namespace diagrammar

#endif  // RUNTIME_GEOMETRY_TRIANGULATE_H_
