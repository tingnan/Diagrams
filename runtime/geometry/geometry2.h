// Copyright 2015 Native Client Authors.

#ifndef RUNTIME_GEOMETRY_GEOMETRY2_H_
#define RUNTIME_GEOMETRY_GEOMETRY2_H_

#include <string>
#include <vector>

#include "include/matrix_types.h"

namespace diagrammar {


struct TriangleMesh {
  std::vector<Vector2f> vertices;
  // each face stores 3 indices to the actual point in the vertices vector
  std::vector<std::array<size_t, 3> > faces;
};


typedef std::vector<Vector2f> Path;

struct Polygon {
  Path path;
  std::vector<Path> holes;
  Polygon() = default;
  explicit Polygon (Path p): path(std::move(p)) {}
};

// Simplify a polyline with a relative tolerance
Path SimplifyPolyline(
    const Path& polyline, float rel_tol = 0.005);

// Triangulates a polygon described by a closed polyline.
// The polyline must not be self intersecting
TriangleMesh TriangulatePolygon(
    const Polygon& polygon);

// Inflates and triangulates an open polyline.
// Inflate by the offset amount (same unit as the input polyline).
TriangleMesh TriangulatePolyline(
    const Path& polyline, float offset);

std::vector<Polygon> DecomposePolygonToConvexhulls(const Polygon& polygon); 

// Using HACD to decompose a complex polygon to convexhulls

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
bool ResolveIntersections(Path& polyline,
                          std::vector<Path>& holes
                          );

}  // namespace diagrammar

#endif  // RUNTIME_GEOMETRY_GEOMETRY2_H_
