// Copyright 2015 Native Client authors
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

  Node();
  Node(const Node&) = delete;
  Node& operator=(const Node&) = delete;
  Node(Node&&) = default;
  Node& operator=(Node&&) = default;
  // using the move stuff
  explicit Node(ComplexShape2D geo);

  // manipulate geometry
  const ComplexShape2D* GetGeometry(unsigned) const;
  ComplexShape2D* GetGeometry(unsigned);
  void AddGeometry(ComplexShape2D geo);
  unsigned GetGeometryCount() const { return collision_shapes_.size(); }

  // get unique id
  int GetID() const;
  void SetID(int id);
  int GetCollisionGroupID() const;
  void SetCollisionGroupID(int id);

  // get coordinate information
  float GetRotationAngle();
  Matrix2f GetRotationMatrix();
  void SetRotationAngle(float angle);
  void SetRotationMatrix(const Matrix2f&);
  void Rotate(float angle);
  Vector2f GetPosition();
  void SetPosition(const Vector2f&);
  void Translate(const Vector2f&);

 private:
  // Geometry: note, we may allow composite geometries
  // we can have a vector of collisionshapes
  // attached to the same PhysicsObject
  std::vector<std::unique_ptr<ComplexShape2D> > collision_shapes_;
  // frame
  CoordinateFrame2D coordinate_;
  // physics property
  PhysicsParams properties_;
  // the unique ID (managed by World)
  int id_ = 0xffffffff;
  // the collision filtering ID, used for broad phase collision filtering only
  int collision_group_id_;
};

}  // namespace diagrammar

#endif  // RUNTIME_PHYSICS_NODE_H_
