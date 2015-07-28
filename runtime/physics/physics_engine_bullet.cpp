// Copyright 2015 Native Client Authors.

#include <iostream>

#include <bullet/btBulletDynamicsCommon.h>

#include "utility/stl_memory.h"
#include "physics/physics_engine_bullet.h"

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
  for (size_t i = btworld_->getNumCollisionObjects() - 1; i != 0; --i) {
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
      node->frame.SetTranslation(Vector2f(pos.getX(), pos.getY()));
      node->frame.SetRotation(quat.getAngle());
      // TODO(tingnan) also add velocity and angular velocity
    }
  }
}

void PhysicsEngineBullet::AddNode(Node* node) {
  assert(body_table_.find(node->id) == body_table_.end());
  body_table_.emplace(node->id, btRigidBodyResource());
  // Create collision shape from polygon and polyline
}

}  // namespace diagrammar
