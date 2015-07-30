// Copyright 2015 Native Client Authors.

#include <iostream>

#include <btBulletDynamicsCommon.h>
#include <BulletCollision/CollisionShapes/btTriangleShape.h>
#include <BulletCollision/CollisionShapes/btConvexHullShape.h>
#include <BulletCollision/CollisionShapes/btCompoundShape.h>

#include "utility/stl_memory.h"
#include "physics/physics_engine_bullet.h"
#include "physics/node.h"
#include "geometry/aabb.h"

namespace {
std::vector<std::unique_ptr<btCollisionShape>> CreateTriangleShapes(
    const diagrammar::TriangleMesh2D& mesh) {
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

std::vector<std::unique_ptr<btCollisionShape>> CreateConvexHullShapes(
    const diagrammar::TriangleMesh2D& mesh, float depth) {
  std::vector<std::unique_ptr<btCollisionShape>> shapes(mesh.faces.size());
  for (size_t i = 0; i < mesh.faces.size(); ++i) {
    shapes[i].reset(new btConvexHullShape());
    btConvexHullShape* hull = static_cast<btConvexHullShape*>(shapes[i].get());
    for (size_t vt_idx = 0; vt_idx < 3; ++vt_idx) {
      auto& vertex = mesh.vertices[mesh.faces[i][vt_idx]];
      hull->addPoint(btVector3(vertex(0), vertex(1), 0.5 * depth));
      hull->addPoint(btVector3(vertex(0), vertex(1), -0.5 * depth));
    }
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
  // Add a ground plane

  ground_plane_.collision_shape.reset(
      new btStaticPlaneShape(btVector3(0, 0, 1), 0));
  btTransform initial_transform;
  initial_transform.setIdentity();
  initial_transform.setOrigin(btVector3(0, 0, -kDepth));
  ground_plane_.motion_state =
      make_unique<btDefaultMotionState>(initial_transform);
  ground_plane_.body.reset(
      new btRigidBody(btRigidBody::btRigidBodyConstructionInfo(
          0, ground_plane_.motion_state.get(),
          ground_plane_.collision_shape.get(), btVector3(0, 0, 0))));
  btworld_->addRigidBody(ground_plane_.body.get());
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
      node->frame.SetTranslation(Vector3f(
          pos.getX() * kScaleUp, pos.getY() * kScaleUp, pos.getZ() * kScaleUp));
      btVector3 rot_axis = quat.getAxis();
      float rot_angle = quat.getAngle();
      node->frame.SetRotation(AngleAxisf(
          rot_angle, Vector3f(rot_axis.x(), rot_axis.y(), rot_axis.z())));
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
    for (auto& shape_ptr : node->collision_shapes) {
      switch (shape_ptr->shape_type) {
        case Shape2DType::kDisk: {
          auto sphere_ptr = dynamic_cast<Disk2D*>(shape_ptr.get());
          rigid_body.child_shapes.emplace_back(
              make_unique<btSphereShape>(sphere_ptr->radius * kScaleDown));
          compound->addChildShape(trans, rigid_body.child_shapes.back().get());
        } break;
        case Shape2DType::kPolygon: {
          auto poly_ptr = dynamic_cast<Polygon2D*>(shape_ptr.get());
          TriangleMesh2D mesh = TriangulatePolygon(*poly_ptr);
          for (auto& v : mesh.vertices) {
            v = kScaleDown * v;
          }
          auto hulls = CreateConvexHullShapes(mesh, kDepth);
          for (auto& hull_ptr : hulls) {
            compound->addChildShape(trans, hull_ptr.get());
          }
          rigid_body.child_shapes.insert(rigid_body.child_shapes.begin(),
                                         std::make_move_iterator(hulls.begin()),
                                         std::make_move_iterator(hulls.end()));
        } break;
        case Shape2DType::kPolyLine: {
          auto line_ptr = dynamic_cast<Line2D*>(shape_ptr.get());
          TriangleMesh2D mesh = TriangulatePolyline(line_ptr->path, 1.5);
          for (auto& v : mesh.vertices) {
            v = kScaleDown * v;
          }
          auto hulls = CreateConvexHullShapes(mesh, kDepth);
          for (auto& hull_ptr : hulls) {
            compound->addChildShape(trans, hull_ptr.get());
          }
          rigid_body.child_shapes.insert(rigid_body.child_shapes.begin(),
                                         std::make_move_iterator(hulls.begin()),
                                         std::make_move_iterator(hulls.end()));
        } break;
        default:
          break;
      }
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

  btTransform initial_transform;
  Vector3f pos = node->frame.GetTranslation();
  initial_transform.setOrigin(
      btVector3(pos(0) * kScaleDown, pos(1) * kScaleDown, pos(2) * kScaleDown));
  Eigen::Quaternionf quat = node->frame.GetRotation();
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
  rigid_body.body->setAngularVelocity(btVector3(node->angular_velocity(0),
                                                node->angular_velocity(1),
                                                node->angular_velocity(2)));
  // Add the body to the btworld;
  btworld_->addRigidBody(rigid_body.body.get());
  // Create the reverse mapping.
  body_node_table_[rigid_body.body.get()] = node;
}

// TODO(tingan) implement each of those
void PhysicsEngineBullet::AddJoint(Joint* joint) {
  if (RevoluteJoint* revo_joint = dynamic_cast<RevoluteJoint*>(joint)) {
    RevoluteJoint joint_info(*revo_joint);
    joint_info.local_anchor_1 = kScaleDown * joint_info.local_anchor_1;
    joint_info.local_anchor_2 = kScaleDown * joint_info.local_anchor_2;
    btVector3 axis_in_both(0, 0, 1);
    btVector3 pivot_1(joint_info.local_anchor_1(0),
                      joint_info.local_anchor_1(1), 0);
    btVector3 pivot_2(joint_info.local_anchor_2(0),
                      joint_info.local_anchor_2(1), 0);
    joint_table_[joint_info.id].reset(
        new btHingeConstraint(*(body_table_[joint_info.node_1].body),
                              *(body_table_[joint_info.node_2].body), pivot_1,
                              pivot_2, axis_in_both, axis_in_both));
    btHingeConstraint* bt_hinge_joint =
        dynamic_cast<btHingeConstraint*>(joint_table_[joint_info.id].get());
    if (joint_info.enable_limit_min || joint_info.enable_limit_max)
      bt_hinge_joint->setLimit(joint_info.angle_min, joint_info.angle_max, 1.0,
                               1.0, 0.0);
    btworld_->addConstraint(bt_hinge_joint, true);
  }
}
void PhysicsEngineBullet::RemoveNodeByID(id_t id) {
  auto body_itr = body_table_.find(id);
  if (body_itr != body_table_.end()) {
    btRigidBody* body = body_itr->second.body.get();
    // destroy all joints attached to the body first
    std::vector<id_t> joint_ids;
    for (auto joint_itr = joint_table_.begin(); joint_itr != joint_table_.end();
         ++joint_itr) {
      auto joint_ptr = joint_itr->second.get();
      btRigidBody& body_a = joint_ptr->getRigidBodyA();
      btRigidBody& body_b = joint_ptr->getRigidBodyB();
      if (&body_a == body || &body_b == body) {
        btworld_->removeConstraint(joint_ptr);
        joint_ids.emplace_back(joint_itr->first);
      }
    }
    // Release the joint memory
    for (auto id : joint_ids) {
      joint_table_.erase(id);
    }

    // Now clean the body
    btworld_->removeRigidBody(body);
    body_table_.erase(body_itr);
  }
}
void PhysicsEngineBullet::RemoveJointByID(id_t id) {
  auto joint_itr = joint_table_.find(id);
  if (joint_itr != joint_table_.end()) {
    std::cerr << "joint distroyed\n";
    btTypedConstraint* joint = joint_itr->second.get();
    btworld_->removeConstraint(joint);
    joint_table_.erase(joint_itr);
  }
}

void PhysicsEngineBullet::ApplyForceToNode(id_t id, const Vector3f& force,
                                           const Vector3f& offset) {}
void PhysicsEngineBullet::ApplyImpulseToNode(id_t id, const Vector3f& impulse,
                                             const Vector3f& offset) {}
void PhysicsEngineBullet::ApplyTorqueToNode(id_t id, const Vector3f& torque) {}
void PhysicsEngineBullet::ApplyAngularImpulseToNode(id_t id,
                                                    const Vector3f& torque) {}
}  // namespace diagrammar
