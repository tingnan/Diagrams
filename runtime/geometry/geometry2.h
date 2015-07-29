// Copyright 2015 Native Client Authors.

#ifndef RUNTIME_GEOMETRY_GEOMETRY2_H_
#define RUNTIME_GEOMETRY_GEOMETRY2_H_

#include <array>
#include <string>
#include <vector>

#include <json/json.h>
#include "include/matrix_types.h"

namespace diagrammar {

struct TriangleMesh {
  std::vector<Vector2f> vertices;
  // Each face stores 3 indices to the actual vertex in the "vertices" vector
  // Warning: clang-newlib cannot handle std::array correctly right now
  std::vector<std::array<size_t, 3> > faces;
};

typedef std::vector<Vector2f> Path;

enum class ShapeType { kNone, kDisk };

// typedef std::underlying_type<ShapeType>::type ShapeTypeInteral;

// We can have different choices for this metadata: boost::any or Json::Value
typedef Json::Value Metadata;

struct Polygon {
  Metadata shape_info;
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

// The method takes an input polygon, then:
// - detect the self intersections in its boundary path and resolve them.
// - detect the self intersections in all the holes and resolve them.
// - union all the holes.
// - clip the polygon with holes (using diff operation)
// - the output maybe a single polygon (with holes) or a vector of polygons
//   if the holes cut the polygon into separate pieces.
std::vector<Polygon> ResolveIntersections(const Polygon& polygon);

}  // namespace diagrammar

#endif  // RUNTIME_GEOMETRY_GEOMETRY2_H_
