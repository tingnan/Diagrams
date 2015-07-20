// Copyright 2015 Native Client authors.

#ifndef RUNTIME_PHYSICS_NODE_H_
#define RUNTIME_PHYSICS_NODE_H_

#include <json/json.h>

#include <vector>

#include "geometry/geometry2.h"
#include "geometry/coordinate_frame.h"

namespace diagrammar {

typedef int id_t;

struct MaterialProperty {
  float restitution = 0.7;
  float friction = 0.3;
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
  virtual ~Joint() = default;
  id_t id = 0xffffffff;
  // Only maintain an id to the node
  id_t node_1 = 0xffffffff;
  id_t node_2 = 0xffffffff;
  // The local anchor set the position of the joint in the node's local frame.
  // With both anchors we can define the relative spatial configuration of the
  // two connected nodes.
  Vector2f local_anchor_1;
  Vector2f local_anchor_2;
  bool enable_limit_min = false;
  bool enable_limit_max = false;
  bool enable_motor = false;
};

struct RevoluteJoint : Joint {
  // The limit to the rotation angle
  ~RevoluteJoint() = default;
  float angle_min;
  float angle_max;
  float motor_speed;
  float max_motor_torque;
};

struct PrismaticJoint : Joint {
  ~PrismaticJoint() = default;
  // Along which direction the two nodes can move relative to each other
  Vector2f local_axis_0;
  // The displacement limit along the local_axis_0
  float translation_min;
  float translation_max;
  float motor_speed;
  float max_motor_force;
};

}  // namespace diagrammar

#endif  // RUNTIME_PHYSICS_NODE_H_
