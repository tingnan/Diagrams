// Copyright 2015 Native Client authors
#ifndef RUNTIME_GEOMETRY_TRIANGULATE_H_
#define RUNTIME_GEOMETRY_TRIANGULATE_H_
#include <vector>
#include "include/typedefs.h"

namespace diagrammar {
struct Triangle2D;
enum PolylineMethod { kDouglasPeucker };
std::vector<Vec2f> Simplify(
    const std::vector<Vec2f>& in,
    PolylineMethod m = PolylineMethod::kDouglasPeucker);
std::vector<Triangle2D> DelaunayTriangulation(const std::vector<Vec2f>& path);
std::vector<Triangle2D> DelaunayTriangulation(
    const std::vector<Vec2f>& path,
    const std::vector<std::vector<Vec2f> >& holes);
std::vector<Triangle2D> InflateAndTriangulate(const std::vector<Vec2f>& path);
}  // namespace diagrammar

#endif  // RUNTIME_GEOMETRY_TRIANGULATE_H_
