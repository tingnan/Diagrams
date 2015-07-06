// Copyright 2015 Native Client Authors.

#ifndef RUNTIME_GEOMETRY_GEOMETRY2_H_
#define RUNTIME_GEOMETRY_GEOMETRY2_H_

#include <vector>
#include "include/matrix_types.h"

namespace diagrammar {

struct Triangle {
  Vector2f p0;
  Vector2f p1;
  Vector2f p2;
};

typedef std::vector<Vector2f> Polyline;

struct Polygon {
  Polyline path;
  std::vector<Polyline> holes;
  Polygon() = default;
  explicit Polygon (Polyline p): path(std::move(p)) {}
};

// TODO(tingnan) document
std::vector<Vector2f> SimplifyPolyline(
    const Polyline& polyline, float rel_tol = 0.005);

// Triangulates a polygon described by a closed polyline.
// The polyline must not be self intersecting
std::vector<Triangle> TriangulatePolygon(
    const Polygon& polygon);

// Inflates and triangulates an open polyline.
// Inflate by the offset amount (same unit as the input polyline).
std::vector<Triangle> TriangulatePolyline(
    const Polyline& polyline, float offset);

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

#endif  // RUNTIME_GEOMETRY_GEOMETRY2_H_
