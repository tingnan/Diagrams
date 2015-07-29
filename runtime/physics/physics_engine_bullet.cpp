// Copyright 2015 Native Client Authors.

#include <iostream>

#include <btBulletDynamicsCommon.h>
#include <BulletCollision/CollisionShapes/btTriangleShape.h>
#include <BulletCollision/CollisionShapes/btCompoundShape.h>

#include "utility/stl_memory.h"
#include "physics/physics_engine_bullet.h"
#include "geometry/aabb.h"

namespace {
std::vector<std::unique_ptr<btCollisionShape>> CreateTriangleShapes(
    const diagrammar::TriangleMesh& mesh) {
  std::vector<std::unique_ptr<btCollisionShape>> shapes(mesh.faces.size());
  for (size_t i = 0; i < mesh.faces.size(); ++i) {
    btVector3 vertices[3];
    for (size_t vt_idx = 0; vt_idx < 3; ++vt_idx) {
      auto& vertex = mesh.vertices[mesh.faces[i][vt_idx]];
      vertices[vt_idx] = btVector3(vertex(0), vertex(1), 0);
    }
    shapes[i].reset(new btTriangleShape(vertices[0], vertices[1], vertices[2]));
  }
  return shapes;
}
}  // namespace

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
      btVector3 rot_axis = quat.getAxis();
      float rot_angle = quat.getAngle();
      if (rot_axis.z() < 0) {
        rot_angle = -rot_angle;
      }
      node->frame.SetRotation(rot_angle);
      // TODO(tingnan) also add velocity and angular velocity
    }
  }
}

void PhysicsEngineBullet::AddNode(Node* node) {
  // hack: ignore the board first
  assert(body_table_.find(node->id) == body_table_.end());
  btRigidBodyResource& rigid_body = body_table_[node->id];

  if (true) {
    rigid_body.collision_shape.reset(new btCompoundShape());
    btCompoundShape* compound =
        static_cast<btCompoundShape*>(rigid_body.collision_shape.get());
    btTransform trans;
    trans.setIdentity();
    for (auto& polygon : node->polygons) {
      if (polygon.shape_info.isMember("type")) {
        auto shape_type = polygon.shape_info["type"].asInt();
        if (static_cast<ShapeType>(shape_type) == ShapeType::kDisk) {
          float radius = polygon.shape_info["radius"].asFloat() * kScaleDown;
          rigid_body.child_shapes.emplace_back();
          rigid_body.child_shapes.back().reset(new btSphereShape(radius));
          compound->addChildShape(trans, rigid_body.child_shapes.back().get());
          continue;
        }
      }

      TriangleMesh mesh = TriangulatePolygon(polygon);
      for (auto& v : mesh.vertices) {
        v = kScaleDown * v;
      }
      auto shapes = CreateTriangleShapes(mesh);
      for (auto& shape_ptr : shapes) {
        compound->addChildShape(trans, shape_ptr.get());
      }
      rigid_body.child_shapes.insert(rigid_body.child_shapes.begin(),
                                     std::make_move_iterator(shapes.begin()),
                                     std::make_move_iterator(shapes.end()));
    }

    for (auto& path : node->paths) {
      // Expand by 1.5 unit
      TriangleMesh mesh = TriangulatePolyline(path, 1.5);
      for (auto& v : mesh.vertices) {
        v = kScaleDown * v;
      }
      auto shapes = CreateTriangleShapes(mesh);
      for (auto& shape_ptr : shapes) {
        compound->addChildShape(trans, shape_ptr.get());
      }
      rigid_body.child_shapes.insert(rigid_body.child_shapes.begin(),
                                     std::make_move_iterator(shapes.begin()),
                                     std::make_move_iterator(shapes.end()));
    }
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
  rigid_body.body->setLinearVelocity(btVector3(
      node->velocity(0) * kScaleDown, node->velocity(1) * kScaleDown, 0));
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
