// Copyright 2015 Native Client Authors.

#ifndef RUNTIME_GEOMETRY_GEOMETRY2_H_
#define RUNTIME_GEOMETRY_GEOMETRY2_H_

#include <string>
#include <vector>

#include "include/matrix_types.h"

namespace diagrammar {

struct TriangleMesh {
  std::vector<Vector2f> vertices;
  // Each face stores 3 indices to the actual vertex in the "vertices" vector
  std::vector<std::array<size_t, 3> > faces;
};

typedef std::vector<Vector2f> Path;

struct Polygon {
  Path path;
  std::vector<Path> holes;
  Polygon() = default;
  explicit Polygon(Path p) : path(std::move(p)) {}
};

// Simplify a path with a relative tolerance
Path SimplifyPolyline(const Path& path, float rel_tol = 0.005);

// Triangulates a polygon described by a closed path.
// The path must not be self intersecting
TriangleMesh TriangulatePolygon(const Polygon& polygon);

// Inflates and triangulates an open path.
// Inflate by the offset amount (same unit as the input path).
TriangleMesh TriangulatePolyline(const Path& path, float offset);

// Use V-HACD to decompose a complex polygon to convexhulls
std::vector<Polygon> DecomposePolygonToConvexhulls(const Polygon& polygon);

// The method takes a boundary path and a set of holes as input, then:
// - detect the self intersection in the path
// - union all the holes
// - execute a difference operation on the polygon using the holes
// - the input path and holes are then modified in place.
// The return value indicates if the operations succeeded.
// The method will fail when:
// - the boundary path is self intersecting
// - the holes split or erase the polygon

// TODO implementation
bool ResolveIntersections(Path& path, std::vector<Path>& holes);

}  // namespace diagrammar

#endif  // RUNTIME_GEOMETRY_GEOMETRY2_H_
