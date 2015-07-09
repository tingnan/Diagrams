// Copyright 2015 Native Client Authors.

#include <stack>
#include <iostream>
#include <utility>
#include <algorithm>
#include <unordered_map>
#include <vector>

#include <v-hacd/VHACD.h>

#include "geometry/geometry2.h"
#include "poly2tri/poly2tri.h"
#include "polyclipping/clipper.hpp"
#include "geometry/aabb.h"

namespace {

inline double dot(const diagrammar::Vector2f& a,
                  const diagrammar::Vector2f& b) {
  return a.adjoint() * b;
}

bool IsApproxZero(double t) {
  const double tol = 1e-8;
  return fabs(t) < tol;
}

// the dist is between the segment [st, ed] and the point pt
// use double precision to prevent overflow (though not likely to happen)
double SqrdDistSegmentToPoint(const diagrammar::Vector2f& bg,
                              const diagrammar::Vector2f& ed,
                              const diagrammar::Vector2f& pt) {
  // first we check the projection from pt to [st, end];
  diagrammar::Vector2f seg = ed - bg;
  diagrammar::Vector2f pt2bg = pt - bg;
  double projected_length = dot(seg, pt2bg);
  double squared_seg_length = dot(seg, seg);

  if (projected_length < 0) {
    return dot(pt2bg, pt2bg);
  }

  if (projected_length > squared_seg_length) {
    diagrammar::Vector2f pt2ed = pt - ed;
    return dot(pt2ed, pt2ed);
  }

  // the segment is degenerate (ed is too close to bg)!
  if (IsApproxZero(squared_seg_length)) return dot(pt2bg, pt2bg);

  return dot(pt2bg, pt2bg) -
         projected_length * (projected_length / squared_seg_length);
}

// the clipper library takes only integer points for robustness
// for floating points we have to scale them up (multiply by 1000 to keep at
// most three digits) and convert them into clipper path(s).
// after clipper processing we can then scale down the points;
const float kUScale = 1000.f;
const float kDScale = 0.001f;

ClipperLib::Path UScalePathDiaToClipper(
    const diagrammar::Path& path) {
  ClipperLib::Path scaled_path;
  scaled_path.reserve(path.size());
  for (const auto& pt : path) {
    scaled_path.emplace_back(
        ClipperLib::IntPoint(pt(0) * kUScale, pt(1) * kUScale));
  }
  return scaled_path;
}

std::vector<p2t::Point> DScalePathClipperToPoly(const ClipperLib::Path& path) {
  std::vector<p2t::Point> out;
  out.reserve(path.size());
  for (const auto& pt : path) {
    out.emplace_back(pt.X * kDScale, pt.Y * kDScale);
  }
  return out;
}

diagrammar::Path DScalePathClipperToDia(
    const ClipperLib::Path& path) {
  diagrammar::Path out;
  out.reserve(path.size());
  for (const auto& pt : path) {
    out.emplace_back(pt.X * kDScale, pt.Y * kDScale);
  }
  return out;
}


// Wrapper of Clipper CleanPolygon for poly2tri to use
// The function does not check for self intersections!
std::vector<p2t::Point> CleanPolygon(
    const diagrammar::Path& path) {
  ClipperLib::Path scaled_path = UScalePathDiaToClipper(path);
  ClipperLib::CleanPolygon(scaled_path);
  return DScalePathClipperToPoly(scaled_path);
}

std::vector<std::vector<p2t::Point> > CleanPolygon(
    const std::vector<diagrammar::Path>& paths) {
  std::vector<std::vector<p2t::Point> > out;
  out.reserve(paths.size());
  for (const auto& path : paths) {
    out.emplace_back(CleanPolygon(path));
  }
  return out;
}

// time complexity NlogN with worst case O(N^2)
// where N is the size of input polyline
void PolylineDouglasPeuckerRecursive(
    const size_t bg, const size_t ed,
    const diagrammar::Path& polyline, float tol,
    std::vector<bool>* out) {
  if (bg == ed - 1) return;
  double max_dist = 0;
  size_t mid = bg + 1;
  for (size_t i = bg + 1; i < ed; ++i) {
    // compute the sqrd distance of every vertices
    double dist =
        SqrdDistSegmentToPoint(polyline[bg], polyline[ed], polyline[i]);
    if (dist > max_dist) {
      max_dist = dist;
      mid = i;
    }
  }
  if (max_dist > tol * tol) {
    PolylineDouglasPeuckerRecursive(bg, mid, polyline, tol, out);
    PolylineDouglasPeuckerRecursive(mid, ed, polyline, tol, out);

  } else {
    // disgard every point between bg and ed
    for (size_t i = bg + 1; i < ed; ++i) {
      (*out)[i] = false;
    }
  }
}

diagrammar::Path PolylineDouglasPeuckerIterative(
    const diagrammar::Path& polyline, float tol) {
  if (polyline.size() <= 2) {
    return polyline;
  }

  std::vector<bool> toKeep(polyline.size(), true);
  size_t bg = 0;
  size_t ed = polyline.size() - 1;
  std::stack<std::pair<size_t, size_t> > indexStack;
  indexStack.emplace(std::make_pair(bg, ed));

  while (!indexStack.empty()) {
    const std::pair<size_t, size_t> tmp = indexStack.top();
    indexStack.pop();

    bg = tmp.first;
    ed = tmp.second;
    if (bg == ed - 1) continue;
    double max_dist = 0;
    size_t mid = bg + 1;
    for (size_t i = bg + 1; i < ed; ++i) {
      double dist =
          SqrdDistSegmentToPoint(polyline[bg], polyline[ed], polyline[i]);
      if (dist > max_dist) {
        max_dist = dist;
        mid = i;
      }
    }
    if (max_dist > tol * tol) {
      indexStack.emplace(std::make_pair(mid, ed));
      indexStack.emplace(std::make_pair(bg, mid));
    } else {
      // disgard every point between bg and ed
      for (size_t i = bg + 1; i < ed; ++i) {
        toKeep[i] = false;
      }
    }
  }

  diagrammar::Path out;
  out.reserve(polyline.size());
  for (size_t i = 0; i < polyline.size(); ++i) {
    if (toKeep[i]) {
      out.emplace_back(polyline[i]);
    }
  }
  return out;
}

diagrammar::TriangleMesh DelaunaySweepline(
    const diagrammar::Path& path,
    const std::vector<diagrammar::Path>& holes) {
  assert(path.size() >= 3);

  std::vector<p2t::Point> polyline = CleanPolygon(path);
  std::vector<p2t::Point*> polylineptr(polyline.size());
  for (size_t i = 0; i < polyline.size(); ++i) {
    polylineptr[i] = &polyline[i];
  }
  p2t::CDT cdt(polylineptr);

  std::vector<std::vector<p2t::Point> > polyhole = CleanPolygon(holes);
  for (size_t i = 0; i < polyhole.size(); ++i) {
    std::vector<p2t::Point*> polyholeptr(polyhole[i].size());
    for (size_t j = 0; j < polyhole[i].size(); ++j) {
      polyholeptr[j] = &polyhole[i][j];
    }
    cdt.AddHole(polyholeptr);
  }
  // after we add boundary and holes, begin triangulation
  // NlogN complexity
  cdt.Triangulate();
  std::vector<p2t::Triangle*> triangles;

  // get the list of triangles, the triangle resources
  // are managed by the cdt object
  triangles = cdt.GetTriangles();

  size_t num_vertices = polyline.size();
  for (auto& hole : polyhole) {
    num_vertices += hole.size();
  }

  std::unordered_map<p2t::Point*, size_t> pt2index;
  diagrammar::TriangleMesh mesh;
  mesh.vertices.reserve(num_vertices);
  mesh.faces.resize(triangles.size());
  for (size_t i = 0; i < triangles.size(); ++i) {
    for (size_t vt_idx = 0; vt_idx < 3; ++vt_idx) {
      p2t::Point* vertex = triangles[i]->GetPoint(vt_idx);
      if (pt2index.find(vertex) != pt2index.end()) {
        mesh.faces[i][vt_idx] = pt2index[vertex];
      } else {
        // add a new point
        mesh.vertices.emplace_back(vertex->x, vertex->y);
        pt2index[vertex] = mesh.vertices.size() - 1;
        mesh.faces[i][vt_idx] = pt2index[vertex];
      }
    }
  }
  return mesh;
}

// checking for collinear/degenerate case
int CounterClockwise(const diagrammar::Vector2f& a,
                     const diagrammar::Vector2f& b,
                     const diagrammar::Vector2f& c) {
  diagrammar::Vector3f atob(b(0) - a(0), b(1) - a(1), 0);
  diagrammar::Vector3f btoc(c(0) - b(0), c(1) - b(1), 0);
  float crossProduct = atob.cross(btoc)(2);
  if (IsApproxZero(crossProduct)) return 0;
  if (crossProduct > 0) return 1;
  return -1;
}

// checking for collinear/degenerate case
int PointInCircumcenter(const diagrammar::Vector2f& a,
                        const diagrammar::Vector2f& b,
                        const diagrammar::Vector2f& c,
                        const diagrammar::Vector2f& p) {
  // a, b, c in counter clockwise order!
  if (!CounterClockwise(a, b, c)) exit(-1);

  diagrammar::Matrix4f Det;
  Det << a(0), a(1), a.adjoint() * a, 1, b(0), b(1), b.adjoint() * b, 1, c(0),
      c(1), c.adjoint() * c, 1, p(0), p(1), p.adjoint() * p, 1;
  float determinant = Det.determinant();
  if (IsApproxZero(determinant)) return 0;
  if (determinant > 0) return 1;
  return -1;
}

}  // namespace

namespace diagrammar {

std::vector<Vector2f> SimplifyPolyline(const Path& polyline) {
  if (polyline.size() <= 2) {
    return polyline;
  }

  std::vector<Vector2f> out;
  if (true) {
    // we compute the boundary square size of the polyline
    AABB bound = GetAABBWithPadding(polyline, 0);
    Vector2f span = bound.upper_bound - bound.lower_bound;
    // set a tolerance to remove points that are too close to
    // each other. the relative tolerance is 0.5% of max size
    // so the simplified curve is still pretty smooth
    float rel_tol = 5e-3;
    float tol = std::max(span(0), span(1)) * rel_tol;
    out = PolylineDouglasPeuckerIterative(polyline, tol);
  } else {
    // only one method implemented
    // the douglas_peucker method may change the topology of input curves
    assert(0);
  }
  return out;
}


TriangleMesh TriangulatePolygon(const Polygon& polygon) {
  return DelaunaySweepline(polygon.path, polygon.holes);
}

TriangleMesh TriangulatePolyline(
    const Path& polyline, float offset) {
  // clipper only works with integer points
  ClipperLib::Path scaled_polyline = UScalePathDiaToClipper(polyline);

  // inflate
  ClipperLib::ClipperOffset co;
  co.AddPath(scaled_polyline, ClipperLib::JoinType::jtMiter,
             ClipperLib::EndType::etOpenButt);
  ClipperLib::Paths inflated;
  // inflate by input amount.
  co.Execute(inflated, offset * kUScale);

  // one out path
  // for now, we do not allow the holes to appear in the inflated path
  // maybe we allow this later
  assert(inflated.size() == 1 && "inflated path has holes");
  std::vector<Vector2f> pts = DScalePathClipperToDia(inflated[0]);
  return DelaunaySweepline(pts, std::vector<Path>());
}

// the execution is expensive, we should not call it frequently
bool ResolveIntersections(Path& polyline,
                          std::vector<Path>& holes) {

  // deep union
  ClipperLib::Path scaled_polyline = UScalePathDiaToClipper(polyline);
  ClipperLib::Clipper clipper;
  clipper.AddPath(scaled_polyline, ClipperLib::ptSubject, true);
  ClipperLib::Paths intermediate_polylines;
  clipper.Execute(ClipperLib::ctUnion, intermediate_polylines, ClipperLib::pftNonZero, ClipperLib::pftNonZero);
  clipper.Clear();

  for (auto& line : intermediate_polylines) {
    if (ClipperLib::Orientation(line)) {
      ClipperLib::ReversePath(line);
    }
  }
  clipper.AddPaths(intermediate_polylines, ClipperLib::ptSubject, true);
  ClipperLib::Paths unioned_polylines;
  clipper.StrictlySimple(true);
  clipper.Execute(ClipperLib::ctUnion, unioned_polylines, ClipperLib::pftNonZero, ClipperLib::pftNonZero);
  clipper.Clear();

  if (unioned_polylines.size() > 1) {
    return false;
  }

  polyline = DScalePathClipperToDia(unioned_polylines[0]);

  // now do a deep union of all the holes
  for (auto& hole : holes) {
    ClipperLib::Path scaled_hole;
  }

  return true;
}

std::vector<Polygon> DecomposePolygonToConvexhulls(const Polygon& polygon) {
  using VHACD::IVHACD;
  std::vector<Polygon> polygons;
  
  // IVHACD::Parameters params;
  // IVHACD* interface = VHACD::CreateVHACD();
  
  return polygons;
}


}  // namespace diagrammar
