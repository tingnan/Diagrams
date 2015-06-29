// Copyright 2015 Native Client Authors.

#ifndef RUNTIME_GEOMETRY_TRIANGULATE_H_
#define RUNTIME_GEOMETRY_TRIANGULATE_H_

#include <vector>

#include <polyclipping/clipper.hpp>
#include "include/matrix_types.h"

namespace diagrammar {

struct Triangle2D;

// currently only one method is supported
enum PolylineMethod { kDouglasPeucker };

std::vector<Vector2f> SimplifyPolyline(
    const std::vector<Vector2f>& in,
    PolylineMethod m = PolylineMethod::kDouglasPeucker);

// input is the boundary polyline
std::vector<Triangle2D> TriangulatePolygon(
    const std::vector<Vector2f>& polyline);

// triangulate a complex polygon with holes in it
std::vector<Triangle2D> TriangulatePolygon(
    const std::vector<Vector2f>& polyline,
    const std::vector<std::vector<Vector2f> >& holes);

// inflate a polyline and then triangulate it
std::vector<Triangle2D> TriangulatePolyline(
    const std::vector<Vector2f>& polyline);

// the method takes a boundary polyline and a set of holes as input, then:
// 1, detect the self intersection in the polyline
// 2, union all the holes
// 3, execute a difference operation on path using the holes
// the input polyline and holes are then modified in place.
// the method can fail when:
// 1, the boundary path is self intersecting
// 2, the holes split or erase the polyline
// TODO implementation
bool ResolveIntersections(std::vector<Vector2f>& polyline,
                          std::vector<std::vector<Vector2f> >& holes
                          );

}  // namespace diagrammar

#endif  // RUNTIME_GEOMETRY_TRIANGULATE_H_
