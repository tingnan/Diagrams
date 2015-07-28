// Copyright 2015 Native Client Authors.

#include <iostream>

#include <bullet/btBulletDynamicsCommon.h>

#include "utility/stl_memory.h"
#include "physics/physics_engine_bullet.h"
#include "geometry/aabb.h"

namespace diagrammar {
PhysicsEngineBullet::PhysicsEngineBullet(float time_step)
    : PhysicsEngine(time_step) {
  bt_collision_config_ = make_unique<btDefaultCollisionConfiguration>();
  bt_collision_dispatcher_ =
      make_unique<btCollisionDispatcher>(bt_collision_config_.get());
  bt_broad_phase_interface_.reset(new btDbvtBroadphase());
  bt_solver_ = make_unique<btSequentialImpulseConstraintSolver>();
  btworld_ = make_unique<btDiscreteDynamicsWorld>(
      bt_collision_dispatcher_.get(), bt_broad_phase_interface_.get(),
      bt_solver_.get(), bt_collision_config_.get());
  btworld_->setGravity(btVector3(0, -2.8, 0));
}

PhysicsEngineBullet::~PhysicsEngineBullet() {}

void PhysicsEngineBullet::Step() { btworld_->stepSimulation(time_step_); }

void PhysicsEngineBullet::SendDataToWorld() {
  for (int i = btworld_->getNumCollisionObjects() - 1; i >= 0; --i) {
    btCollisionObject* obj = btworld_->getCollisionObjectArray()[i];
    btRigidBody* body = btRigidBody::upcast(obj);
    // TODO(tingnan) check more details about motion state
    if (body && body->getMotionState() &&
        body_node_table_.find(body) != body_node_table_.end()) {
      btTransform trans;
      body->getMotionState()->getWorldTransform(trans);
      Node* node = body_node_table_.find(body)->second;
      const btVector3& pos = trans.getOrigin();
      btQuaternion quat = trans.getRotation();
      node->frame.SetTranslation(
          Vector2f(pos.getX() * kScaleUp, pos.getY() * kScaleUp));
      node->frame.SetRotation(quat.getAngle());
      // TODO(tingnan) also add velocity and angular velocity
    }
  }
}

void PhysicsEngineBullet::AddNode(Node* node) {
  // hack: ignore the board first
  if (node->id == 0) return;
  assert(body_table_.find(node->id) == body_table_.end());
  btRigidBodyResource& rigid_body = body_table_[node->id];
  // TODO(tingnan) Create collision shape from polygon and polyline
  // cheating with balls first
  {
    AABB aabb;
    for (const auto& poly : node->polygons) {
      aabb = GetAABBWithPadding(poly.path, 0.0);
    }
    for (const auto& path : node->paths) {
      aabb = GetAABBWithPadding(path, 0.0);
    }
    float radius = (aabb.upper_bound - aabb.lower_bound).norm() / 2.0;
    rigid_body.collision_shape.reset(new btSphereShape(radius * kScaleDown));
    // We also modify the original path for now
    node->paths.clear();
    node->polygons.clear();
    const size_t n_pts = 30;
    Path path, hole;
    for (size_t i = 0; i < n_pts; ++i) {
      float angle = M_PI * float(i) * 2.0 / float(n_pts);
      path.emplace_back(radius * cos(angle), radius * sin(angle));
      hole.emplace_back(radius * 0.8 * cos(angle), radius * 0.8 * sin(angle));
    }
    node->polygons.emplace_back(Polygon(path));
    node->polygons[0].holes.emplace_back(hole);
  }
  // Compute inertial matrix.
  float mass = 0;
  if (node->motion_type == MotionType::kDynamic) {
    // TODO(tingan) calculate mass from density
    mass = 1.0;
  }
  btVector3 local_inertia(0, 0, 0);
  rigid_body.collision_shape->calculateLocalInertia(mass, local_inertia);
  // Now create a motion state.
  Vector2f pos = node->frame.GetTranslation();
  float rotation_angle = node->frame.GetRotationAngle();
  Eigen::Quaternionf quat(Eigen::AngleAxisf(rotation_angle, Vector3f::UnitZ()));
  btTransform initial_transform;
  initial_transform.setOrigin(
      btVector3(pos(0) * kScaleDown, pos(1) * kScaleDown, 0));
  initial_transform.setRotation(
      btQuaternion(quat.x(), quat.y(), quat.z(), quat.w()));
  rigid_body.motion_state =
      make_unique<btDefaultMotionState>(initial_transform);
  // Finally create our rigid body.
  rigid_body.body.reset(
      new btRigidBody(btRigidBody::btRigidBodyConstructionInfo(
          mass, rigid_body.motion_state.get(), rigid_body.collision_shape.get(),
          local_inertia)));
  rigid_body.body->setLinearVelocity(btVector3(node->velocity(0) * kScaleDown,
                                               node->velocity(1) * kScaleDown));
  rigid_body.body->setAngularVelocity(btVector3(0, 0, node->angular_velocity));
  // Add the body to the btworld;
  btworld_->addRigidBody(rigid_body.body.get());
  // Create the reverse mapping.
  body_node_table_[rigid_body.body.get()] = node;
}

// TODO(tingan) implement each of those
void PhysicsEngineBullet::AddJoint(Joint* joint) {}
void PhysicsEngineBullet::RemoveNodeByID(id_t id) {}
void PhysicsEngineBullet::RemoveJointByID(id_t id) {}

void PhysicsEngineBullet::ApplyForceToNode(id_t, const Vector2f& force,
                                           const Vector2f& offset) {}
void PhysicsEngineBullet::ApplyImpulseToNode(id_t, const Vector2f& impulse,
                                             const Vector2f& offset) {}
void PhysicsEngineBullet::ApplyTorqueToNode(id_t, float torque) {}
void PhysicsEngineBullet::ApplyAngularImpulseToNode(id_t, float torque) {}
}  // namespace diagrammar
