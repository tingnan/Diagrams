// Copyright 2015 Native Client authors.

#ifndef RUNTIME_PHYSICS_NODE_H_
#define RUNTIME_PHYSICS_NODE_H_

#include <memory>
#include <vector>

#include "geometry/geometry2.h"
#include "geometry/coordinate_frame.h"

namespace diagrammar {

class Node {
 public:
  
  struct PhysicsParams {
    float restitution = 1;
    float friction = 0;
    float mass = 1;
    // in its fixed frame;
    Matrix3f inertiaMatrix = Matrix3f::Identity();
  };

  Node() = default;
  Node(const Node&);
  Node& operator=(Node);
  Node(Node&&) = default;
  Node& operator=(Node&&) = default;
  
  unsigned GetNumPolygon() const { return polygons_.size(); }
  Polygon* GetPolygon(unsigned) const;
  unsigned GetNumPolyline() const { return polylines_.size(); }
  Polyline* GetPolyline(unsigned) const; 
  void AddGeometry(Polygon polygon);
  void AddGeometry(Polyline polyline);
  
  int id() const;
  void set_id(int id);
  int collision_group_id() const;
  void set_collision_group_id(int id);

  float GetRotationAngle() const;
  Matrix2f GetRotationMatrix() const;
  void SetRotationAngle(float angle);
  void SetRotationMatrix(const Matrix2f&);
  void Rotate(float angle);
  Vector2f GetPosition() const;
  void SetPosition(const Vector2f&);
  void Translate(const Vector2f&);

 private:
  void swap(Node& rhs);
  
  std::vector<std::unique_ptr<Polygon> > polygons_;
  std::vector<std::unique_ptr<Polyline> > polylines_;

  CoordinateFrame2D frame_;
  PhysicsParams properties_;
  // the unique ID (managed by World)
  int id_ = 0xffffffff;
  // the collision filtering ID, used for broad phase collision filtering only
  int collision_group_id_;
};

}  // namespace diagrammar

#endif  // RUNTIME_PHYSICS_NODE_H_
