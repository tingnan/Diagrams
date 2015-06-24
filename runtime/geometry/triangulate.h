#ifndef _DIAGRAMMAR_TRIANGULATION_
#define _DIAGRAMMAR_TRIANGULATION_
#include <vector>
#include "typedefs.h"

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
}

#endif