// Copyright 2015 Native Client Authors.

#ifndef RUNTIME_GEOMETRY_AABB_H_
#define RUNTIME_GEOMETRY_AABB_H_

#include <vector>

#include "include/matrix_types.h"

namespace diagrammar {
typedef struct AABB {
  Vector2f lower_bound;
  Vector2f upper_bound;
} AABB;

static AABB GetAABBWithPadding(const std::vector<Vector2f>& closure,
                               float pad_percent) {
  float xmin = closure[0](0);
  float xmax = closure[0](0);
  float ymin = closure[0](1);
  float ymax = closure[0](1);
  for (const auto& pt : closure) {
    if (pt(0) < xmin) xmin = pt(0);
    if (pt(0) > xmax) xmax = pt(0);
    if (pt(1) < ymin) ymin = pt(1);
    if (pt(1) > ymax) ymax = pt(1);
  }

  AABB bound;
  Vector2f padding(pad_percent * (xmax - xmin), pad_percent * (ymax - ymin));
  bound.lower_bound(0) = xmin;
  bound.lower_bound(1) = ymin;
  bound.lower_bound -= padding;
  bound.upper_bound(0) = xmax;
  bound.upper_bound(1) = ymax;
  bound.upper_bound += padding;
  return bound;
}
}  // namespace diagrammar
#endif  // RUNTIME_GEOMETRY_AABB_H_
