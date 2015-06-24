// Copyright 2015 Native Client Authors.
#ifndef RUNTIME_GEOMETRY_TRIANGULATE_H_
#define RUNTIME_GEOMETRY_TRIANGULATE_H_
#include <vector>
#include "include/typedefs.h"

namespace diagrammar {
struct Triangle2D;
enum PolylineMethod { kDouglasPeucker };
// simplify an input curved, using the selected method
std::vector<Vector2f> Simplify(
    const std::vector<Vector2f>& in,
    PolylineMethod m = PolylineMethod::kDouglasPeucker);
// triangulate a complex 2d shape, described by its boundary
std::vector<Triangle2D> DelaunayTriangulation(
    const std::vector<Vector2f>& path);
// triangulate a complex 2d shape with holes
std::vector<Triangle2D> DelaunayTriangulation(
    const std::vector<Vector2f>& path,
    const std::vector<std::vector<Vector2f> >& holes);
// inflate an open path and then triangulate it
std::vector<Triangle2D> InflateAndTriangulate(
    const std::vector<Vector2f>& path);
}  // namespace diagrammar

#endif  // RUNTIME_GEOMETRY_TRIANGULATE_H_
