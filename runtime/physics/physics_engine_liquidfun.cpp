// Copyright 2015 Native Client Authors.

#include <Box2D/Box2D.h>

#include <random>
#include <iostream>
#include <vector>

#include "physics/physics_engine_liquidfun.h"
#include "physics/world.h"
#include "geometry/geometry2.h"

namespace {
b2Fixture* AddShapeToBody(const b2Shape& shape,
                          const diagrammar::MaterialProperty& m_property,
                          b2Body* b) {
  b2FixtureDef shape_fixture;
  shape_fixture.shape = &shape;
  shape_fixture.density = m_property.density;
  shape_fixture.friction = m_property.friction;
  shape_fixture.restitution = m_property.restitution;
  return b->CreateFixture(&shape_fixture);
}

void AddTrianglesToBody(const diagrammar::TriangleMesh2D& mesh,
                        const diagrammar::MaterialProperty& m_property,
                        b2Body* b) {
  for (size_t i = 0; i < mesh.faces.size(); ++i) {
    b2Vec2 vertices[3];
    for (size_t vt_idx = 0; vt_idx < 3; ++vt_idx) {
      auto& vertex = mesh.vertices[mesh.faces[i][vt_idx]];
      vertices[vt_idx].Set(vertex(0), vertex(1));
    }
    b2PolygonShape polygon;
    polygon.Set(vertices, 3);
    AddShapeToBody(polygon, m_property, b);
  }
}

void AddDiskToBody(float radius, const diagrammar::MaterialProperty& m_property,
                   b2Body* b) {
  b2CircleShape disk;
  disk.m_radius = radius;
  AddShapeToBody(disk, m_property, b);
}

b2Joint* AddRevoluteJointToWorld(const diagrammar::RevoluteJoint& joint_info,
                                 b2World* world, b2Body* body1, b2Body* body2) {
  b2RevoluteJointDef pin_def;
  pin_def.bodyA = body1;
  pin_def.bodyB = body2;
  pin_def.collideConnected = false;
  pin_def.localAnchorA.Set(joint_info.local_anchor_1(0),
                           joint_info.local_anchor_1(1));
  pin_def.localAnchorB.Set(joint_info.local_anchor_2(0),
                           joint_info.local_anchor_2(1));
  if (joint_info.enable_limit_min) {
    pin_def.enableLimit = true;
    pin_def.lowerAngle = joint_info.angle_min;
  }

  if (joint_info.enable_limit_max) {
    pin_def.enableLimit = true;
    pin_def.upperAngle = joint_info.angle_max;
  }
  return world->CreateJoint(&pin_def);
}

void ApplyForceToBody(b2Body* body, const diagrammar::Vector2f& force,
                      const diagrammar::Vector2f& offset) {
#ifdef __native_client__
  body->ApplyForce(b2Vec2(force(0), force(1)),
                   body->GetWorldPoint(b2Vec2(offset(0), offset(1))));
#else
  body->ApplyForce(b2Vec2(force(0), force(1)),
                   body->GetWorldPoint(b2Vec2(offset(0), offset(1))), true);
#endif
}

void ApplyImpulseToBody(b2Body* body, const diagrammar::Vector2f& impulse,
                        const diagrammar::Vector2f& offset) {
#ifdef __native_client__
  body->ApplyLinearImpulse(b2Vec2(impulse(0), impulse(1)),
                           body->GetWorldPoint(b2Vec2(offset(0), offset(1))));
#else
  body->ApplyLinearImpulse(b2Vec2(impulse(0), impulse(1)),
                           body->GetWorldPoint(b2Vec2(offset(0), offset(1))),
                           true);

#endif
}

void ApplyTorqueToBody(b2Body* body, float torq) {
#ifdef __native_client__
  body->ApplyTorque(torq);
#else
  body->ApplyTorque(torq, true);
#endif
}
void ApplyAngularImpulseToBody(b2Body* body, float impulse) {
#ifdef __native_client__
  body->ApplyAngularImpulse(impulse);
#else
  body->ApplyAngularImpulse(impulse, true);
#endif
}

}  // namespace

namespace diagrammar {

class Box2DJointDestructionListener : public b2DestructionListener {
 public:
  explicit Box2DJointDestructionListener(
      std::unordered_map<diagrammar::id_t, b2Joint*>* p)
      : table_ptr_(p) {}
  void SayGoodbye(b2Fixture* fixture) {}
  void SayGoodbye(b2Joint* joint) {
    // remove all references to joint.
    diagrammar::Joint* joint_ptr =
        static_cast<diagrammar::Joint*>(joint->GetUserData());
    table_ptr_->erase(joint_ptr->id);
  }

 private:
  std::unordered_map<diagrammar::id_t, b2Joint*>* table_ptr_;
};

PhysicsEngineLiquidFun::PhysicsEngineLiquidFun(float time_step)
    : PhysicsEngine(time_step) {
  // we now try to also create a box2d world
  b2Vec2 gravity(0.f, -2.8f);
  b2world_ = make_unique<b2World>(gravity);
  joint_destruction_listener_ =
      make_unique<Box2DJointDestructionListener>(&joint_table_);
  b2world_->SetDestructionListener(joint_destruction_listener_.get());
}

PhysicsEngineLiquidFun::~PhysicsEngineLiquidFun() {}

void PhysicsEngineLiquidFun::Step() {
  b2world_->Step(time_step_, velocity_iterations_, position_iterations_);
}

void PhysicsEngineLiquidFun::SendDataToWorld() {
  for (b2Body* b = b2world_->GetBodyList(); b; b = b->GetNext()) {
    Node* node = reinterpret_cast<Node*>(b->GetUserData());
    if (node) {
      Vector3f translation(b->GetPosition().x * kScaleUp,
                           b->GetPosition().y * kScaleUp, 0);
      node->frame.SetTranslation(translation);
      node->frame.SetRotation(AngleAxisf(b->GetAngle(), Vector3f(0, 0, 1)));
      node->velocity = Vector3f(b->GetLinearVelocity().x * kScaleUp,
                                b->GetLinearVelocity().y * kScaleUp, 0);
      node->angular_velocity = Vector3f(0, 0, b->GetAngularVelocity());
    }
  }
}

void PhysicsEngineLiquidFun::AddNode(Node* node) {
  assert(body_table_.find(node->id) == body_table_.end() &&
         "node was added before!");

  b2BodyDef body_def;
  Vector3f pos = node->frame.GetTranslation();
  body_def.position.Set(pos(0) * kScaleDown, pos(1) * kScaleDown);
  if (node->id == 13) {
    std::cout << pos(0) * kScaleDown << " " << pos(1) * kScaleDown << std::endl;
  }
  AngleAxisf rotation(node->frame.GetRotation());
  Vector3f axis = rotation.axis();
  body_def.angle = rotation.angle();
  if (axis(2) < 0) {
    body_def.angle = -body_def.angle;
  }
  // TODO(tingnan) Add kinematic body
  if (node->motion_type == MotionType::kDynamic) {
    body_def.type = b2_dynamicBody;
    // only the z component
    body_def.angularVelocity = node->angular_velocity(2);
    body_def.linearVelocity.Set(node->velocity(0) * kScaleDown,
                                node->velocity(1) * kScaleDown);
  }
  b2Body* body = b2world_->CreateBody(&body_def);
  // Add to look up table
  body_table_[node->id] = body;
  // the body will keep a pointer to the node
  body->SetUserData(node);

  // create a set of triangles, for each geometry the node has
  for (auto& shape_ptr : node->collision_shapes) {
    switch (shape_ptr->shape_type) {
      case Shape2DType::kDisk: {
        auto sphere_ptr = dynamic_cast<Disk2D*>(shape_ptr.get());
        AddDiskToBody(sphere_ptr->radius * kScaleDown, node->material_info,
                      body);
      } break;
      case Shape2DType::kPolygon: {
        auto poly_ptr = dynamic_cast<Polygon2D*>(shape_ptr.get());
        TriangleMesh2D mesh = TriangulatePolygon(*poly_ptr);
        for (auto& v : mesh.vertices) {
          v = kScaleDown * v;
        }
        AddTrianglesToBody(mesh, node->material_info, body);
      } break;
      case Shape2DType::kPolyLine: {
        auto line_ptr = dynamic_cast<Line2D*>(shape_ptr.get());
        TriangleMesh2D mesh = TriangulatePolyline(line_ptr->path, 1.5);
        for (auto& v : mesh.vertices) {
          v = kScaleDown * v;
        }
        AddTrianglesToBody(mesh, node->material_info, body);
      } break;
      default:
        break;
    }
  }
}

void PhysicsEngineLiquidFun::RemoveNodeByID(id_t id) {
  auto itr = body_table_.find(id);
  if (itr != body_table_.end()) {
    b2world_->DestroyBody(itr->second);
    body_table_.erase(itr);
  }
}

void PhysicsEngineLiquidFun::AddJoint(Joint* joint) {
  if (RevoluteJoint* revo_joint = dynamic_cast<RevoluteJoint*>(joint)) {
    RevoluteJoint joint_info(*revo_joint);
    joint_info.local_anchor_1 = kScaleDown * joint_info.local_anchor_1;
    joint_info.local_anchor_2 = kScaleDown * joint_info.local_anchor_2;
    b2Joint* new_joint = AddRevoluteJointToWorld(
        joint_info, b2world_.get(), body_table_[revo_joint->node_1],
        body_table_[revo_joint->node_2]);
    joint_table_[joint->id] = new_joint;
    new_joint->SetUserData(joint);
  }
}

void PhysicsEngineLiquidFun::RemoveJointByID(id_t id) {
  auto itr = joint_table_.find(id);
  if (itr != joint_table_.end()) {
    b2world_->DestroyJoint(itr->second);
    joint_table_.erase(itr);
  }
}

void PhysicsEngineLiquidFun::ApplyForceToNode(id_t id, const Vector3f& force,
                                              const Vector3f& offset) {
  auto itr = body_table_.find(id);
  if (itr != body_table_.end()) {
    // Important, when length is scaled down, force also scales down
    // in SI unit force is kg m s^-2
    ApplyForceToBody(itr->second, kScaleDown * force.head<2>(),
                     kScaleDown * offset.head<2>());
  }
}

void PhysicsEngineLiquidFun::ApplyImpulseToNode(id_t id,
                                                const Vector3f& impulse,
                                                const Vector3f& offset) {
  auto itr = body_table_.find(id);
  if (itr != body_table_.end()) {
    ApplyImpulseToBody(itr->second, kScaleDown * impulse.head<2>(),
                       kScaleDown * offset.head<2>());
  }
}

void PhysicsEngineLiquidFun::ApplyTorqueToNode(id_t id,
                                               const Vector3f& torque) {
  auto itr = body_table_.find(id);
  if (itr != body_table_.end()) {
    // When length is scaled, the torque is scaled twice (kg m^2 s^-2)
    ApplyTorqueToBody(itr->second, kScaleDown * kScaleDown * torque(2));
  }
}
void PhysicsEngineLiquidFun::ApplyAngularImpulseToNode(
    id_t id, const Vector3f& impulse) {
  auto itr = body_table_.find(id);
  if (itr != body_table_.end()) {
    ApplyAngularImpulseToBody(itr->second,
                              kScaleDown * kScaleDown * impulse(2));
  }
}

}  // namespace diagrammar
