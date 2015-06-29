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
  explicit Node(const ComplexPolygon& geo);
  explicit Node(const std::vector<ComplexPolygon>& geos);
  Node(const Node&);
  Node& operator=(Node);
  Node(Node&&) = default;
  Node& operator=(Node&&) = default;
  

  const ComplexPolygon* GetGeometry(unsigned) const;
  ComplexPolygon* GetGeometry(unsigned);
  void AddGeometry(ComplexPolygon geo);
  unsigned GetGeometryCount() const { return collision_shapes_.size(); }

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
  // swap the content with rhs
  void swap(Node& rhs);
  std::vector<std::unique_ptr<ComplexPolygon> > collision_shapes_;
  CoordinateFrame2D coordinate_;
  PhysicsParams properties_;
  // the unique ID (managed by World)
  int id_ = 0xffffffff;
  // the collision filtering ID, used for broad phase collision filtering only
  int collision_group_id_;
};

}  // namespace diagrammar

#endif  // RUNTIME_PHYSICS_NODE_H_
