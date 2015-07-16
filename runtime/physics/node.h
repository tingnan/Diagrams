// Copyright 2015 Native Client authors.

#ifndef RUNTIME_PHYSICS_NODE_H_
#define RUNTIME_PHYSICS_NODE_H_

#include <vector>

#include <json/json.h>
#include "geometry/geometry2.h"
#include "geometry/coordinate_frame.h"

namespace diagrammar {

typedef int id_t;


struct MaterialProperty {
  float restitution = 1;
  float friction = 0;
  float density = 1;
  // In its fixed frame;
  Matrix3f inertia_matrix = Matrix3f::Identity();
};

struct Node {

  std::vector<Polygon> polygons;
  std::vector<Path> paths;
  
  CoordinateFrame2D frame;
  Vector2f velocity;
  float angular_velocity;

  MaterialProperty material_info;
    // The unique ID (managed by World)
  id_t id = 0xffffffff;
  // The collision filtering ID, used for broad phase collision filtering only
  id_t collision_group_id;
  // Type of node : stationary or dynamic
  bool is_dynamic = false;
  bool is_collidable = true;
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
