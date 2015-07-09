// Copyright 2015 Native Client authors.

#ifndef RUNTIME_PHYSICS_NODE_H_
#define RUNTIME_PHYSICS_NODE_H_

#include <memory>
#include <vector>

#include <json/json.h>
#include "geometry/geometry2.h"
#include "geometry/coordinate_frame.h"

namespace diagrammar {

typedef int id_t;

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
  unsigned GetNumPath() const { return paths_.size(); }
  Path* GetPath(unsigned) const; 
  void AddGeometry(Polygon polygon);
  void AddGeometry(Path polyline);
  
  id_t id() const { return id_; }
  void set_id(id_t id) { id_ = id; }
  id_t collision_group_id() const { return collision_group_id_; }
  void set_collision_group_id(id_t id) { collision_group_id_ = id; }
  bool is_dynamic() const { return is_dynamic_; };
  void set_dynamic(bool dynamic) { is_dynamic_ = dynamic; } 

  float GetRotationAngle() const;
  Matrix2f GetRotationMatrix() const;
  void SetRotationAngle(float angle);
  void SetRotationMatrix(Matrix2f mat);
  void Rotate(float angle);
  
  Vector2f GetPosition() const;
  void SetPosition(const Vector2f&);
  void Translate(const Vector2f&);

  // Get local velocity
  Vector2f GetVelocity() const;
  float GetAngularVelocity() const;

  void SetVelocity(Vector2f);
  void SetAngularVelocity(float);


 private:
  void swap(Node& rhs);
  
  std::vector<std::unique_ptr<Polygon> > polygons_;
  std::vector<std::unique_ptr<Path> > paths_;
  CoordinateFrame2D frame_;

  PhysicsParams properties_;
  // The unique ID (managed by World)
  id_t id_ = 0xffffffff;
  // The collision filtering ID, used for broad phase collision filtering only
  id_t collision_group_id_;
  // Type of node : stationary or dynamic
  bool is_dynamic_ = false;
};


// Joint

struct Joint {
  id_t id;
  Node* node_ptr_0 = nullptr;
  Node* node_ptr_1 = nullptr;
  // The local anchor set the position of the joint in the node's local frame. 
  // With both anchors we can define the relative spatial configuration of the two 
  // connected nodes.
  Vector2f local_anchor_0;
  Vector2f local_anchor_1;
  bool enable_limit = false;
  bool enable_motor = false;
};

struct RevoluteJoint : Joint {
  // Ahe limit to the rotation angle
  float lower_angle;
  float upper_angle;
  float motor_speed;
  float max_motor_torque;
};

struct PrismaticJoint : Joint {
  // Along which direction the two nodes can move relative to each other
  Vector2f local_axis_0;
  // the displacement limit along the local_axis_0
  float lower_translation;
  float upper_translation;
  float motor_speed;
  float max_motor_force;
};


}  // namespace diagrammar

#endif  // RUNTIME_PHYSICS_NODE_H_
