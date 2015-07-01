// Copyright 2015 Native Client Authors.

#ifndef RUNTIME_GEOMETRY_TRIANGULATE_H_
#define RUNTIME_GEOMETRY_TRIANGULATE_H_

#include <vector>

#include <polyclipping/clipper.hpp>
#include "include/matrix_types.h"

namespace diagrammar {

struct Triangle2D;

std::vector<Vector2f> SimplifyPolyline(
    const std::vector<Vector2f>& in);

// Triangulates a polygon described by a closed polyline.
// The polyline must not be self intersecting
std::vector<Triangle2D> TriangulatePolygon(
    const std::vector<Vector2f>& polyline);

// Triangulates a complex polygon with holes in it.
std::vector<Triangle2D> TriangulatePolygon(
    const std::vector<Vector2f>& polyline,
    const std::vector<std::vector<Vector2f> >& holes);

// Inflates and triangulates an open polyline.
std::vector<Triangle2D> TriangulatePolyline(
    const std::vector<Vector2f>& polyline);

// The method takes a boundary polyline and a set of holes as input, then:
// - detect the self intersection in the polyline
// - union all the holes
// - execute a difference operation on the polygon using the holes
// - the input polyline and holes are then modified in place.
// The return value indicates if the operations succeeded. 
// The method will fail when:
// - the boundary path is self intersecting
// - the holes split or erase the polygon

// TODO implementation
bool ResolveIntersections(std::vector<Vector2f>& polyline,
                          std::vector<std::vector<Vector2f> >& holes
                          );

}  // namespace diagrammar

#endif  // RUNTIME_GEOMETRY_TRIANGULATE_H_
