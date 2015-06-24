#ifndef _DIAGRAMMAR_PHYSICSOBJ_
#define _DIAGRAMMAR_PHYSICSOBJ_

#include "geometry/geometry2.h"
#include "geometry/coordinate_frame.h"
#include <memory>

namespace diagrammar {

class Node {
 public:
  struct PhysicsParams {
    float restitution = 1;
    float friction = 0;
    float mass = 1;
    // in its fixed frame;
    Eigen::Matrix3f inertiaMatrix = Eigen::Matrix3f::Identity();
  };
 
  Node();
  Node(const Node&);
  Node& operator=(const Node&);
  Node(Node&&) = default;
  Node& operator=(Node&&) = default;
  // using the move stuff
  Node(ComplexShape2D geo);

  // manipulate geometry
  const ComplexShape2D* GetGeometry(unsigned) const;
  ComplexShape2D* GetGeometry(unsigned);
  void AddGeometry(ComplexShape2D&& geo);
  unsigned GetGeometryCount() const { return collision_shapes_.size(); }

  // get unique id
  int GetID() const;
  void SetID(int id);
  int GetCollisionGroupID() const;
  void SetCollisionGroupID(int id);

  // get coordinate information
  float GetRotationAngle();
  Mat2f GetRotationMatrix();
  void SetRotationAngle(float);
  void SetRotationMatrix(const Mat2f&);
  void Rotate(float);
  Vec2f GetPosition();
  void SetPosition(const Vec2f&);
  void Translate(const Vec2f&);

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
}

#endif