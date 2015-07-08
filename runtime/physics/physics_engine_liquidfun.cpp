// Copyright 2015 Native Client Authors.

#include <random>
#include <iostream>
#include <vector>

#include "Box2D/Box2D.h"
#include "physics/physics_engine_liquidfun.h"
#include "physics/world.h"
#include "geometry/geometry2.h"

namespace {

void AddRevoluteJointToWorld(b2World* world, b2Body* body1, b2Body* body2) {
  b2RevoluteJointDef pin_def;
  pin_def.bodyA = body1;
  pin_def.bodyB = body2;
  pin_def.collideConnected = false;
  pin_def.localAnchorA.SetZero();
  b2Vec2 pos = body1->GetPosition();
  pin_def.localAnchorB.Set(pos.x, pos.y);
  world->CreateJoint(&pin_def);
}

}  // namespace

namespace diagrammar {

PhysicsEngineLiquidFun::PhysicsEngineLiquidFun(float time_step) : PhysicsEngine(time_step) {
  // we now try to also create a box2d world
  b2Vec2 gravity(0.f, -4.9f);
  b2world_ = new b2World(gravity);
}

void PhysicsEngineLiquidFun::AddTrianglesToBody(std::vector<Triangle> triangles, b2Body* b) {
  for (size_t i = 0; i < triangles.size(); ++i) {
    b2Vec2 vertices[3];
    vertices[0].Set(triangles[i].p0(0) * kScaleDown, triangles[i].p0(1) * kScaleDown);
    vertices[1].Set(triangles[i].p1(0) * kScaleDown, triangles[i].p1(1) * kScaleDown);
    vertices[2].Set(triangles[i].p2(0) * kScaleDown, triangles[i].p2(1) * kScaleDown);
    b2PolygonShape polygon;
    polygon.Set(vertices, 3);
    b2FixtureDef polyfixture;
    polyfixture.shape = &polygon;
    polyfixture.density = kDefaultDensity;
    polyfixture.friction = kDefaultFriction;
    polyfixture.restitution = kDefaultRestitution;
    b->CreateFixture(&polyfixture);
  }
}

void PhysicsEngineLiquidFun::AddNode(Node* node) {
  b2BodyDef body_def;
  Vector2f pos = node->GetPosition();
  body_def.position.Set(pos(0) * kScaleDown, pos(1) * kScaleDown);
  body_def.angle = node->GetRotationAngle();
  if (node->is_dynamic()) {
    body_def.type = b2_dynamicBody;
    body_def.angularVelocity = node->GetAngularVelocity();
    Vector2f velocity = node->GetVelocity();
    body_def.linearVelocity.Set(velocity(0) * kScaleDown, velocity(1) * kScaleDown);
  }
  b2Body* body = b2world_->CreateBody(&body_def);

  // the body will keep a pointer to the node
  body->SetUserData(node);

  // create a set of triangles, for each geometry the node has
  for (unsigned count = 0; count < node->GetNumPolygon(); ++count) {
    Polygon* poly = node->GetPolygon(count);
    AddTrianglesToBody(TriangulatePolygon(*poly), body);
  }

  for (unsigned count = 0; count < node->GetNumPolyline(); ++count) {
    Polyline* line = node->GetPolyline(count);
    // Expand by 1.5 unit
    AddTrianglesToBody(TriangulatePolyline(*line, 1.5), body);
  }

}

PhysicsEngineLiquidFun::~PhysicsEngineLiquidFun() {}

void PhysicsEngineLiquidFun::Step() {
  b2world_->Step(time_step_, velocity_iterations_,
                 position_iterations_);
}

void PhysicsEngineLiquidFun::SendDataToWorld() {
  for (b2Body* b = b2world_->GetBodyList(); b; b = b->GetNext()) {
    Node* node = reinterpret_cast<Node*>(b->GetUserData());
    if (node) {
      Vector2f translation(b->GetPosition().x * kScaleUp,
                           b->GetPosition().y * kScaleUp);
      node->SetPosition(translation);
      node->SetRotationAngle(b->GetAngle());
    }
  }
}

void PhysicsEngineLiquidFun::RemoveNodeByID(id_t id) {

}

/* // The code is not used
void PhysicsEngineLiquidFun::AddNodeToWorld(b2Body* body, World* world) {
  Node* new_node = world->AddNode(Node());
  body->SetUserData(new_node);
  for (b2Fixture* shape_fixture = body->GetFixtureList(); shape_fixture;
       shape_fixture = shape_fixture->GetNext()) {
    b2Shape::Type shape_type = shape_fixture->GetType();

    if (shape_type == b2Shape::e_polygon) {
      b2PolygonShape* shape =
          dynamic_cast<b2PolygonShape*>(shape_fixture->GetShape());
      size_t count = shape->GetVertexCount();
      std::vector<Vector2f> vertices(count);
      for (size_t i = 0; i < count; ++i) {
        b2Vec2 tmp = shape->GetVertex(i);
        vertices[i] = Vector2f(tmp.x, tmp.y) * kScaleUp;
      }
      new_node->AddGeometry(Polygon(vertices));
    }

    if (shape_type == b2Shape::e_circle) {
      b2CircleShape* shape =
          dynamic_cast<b2CircleShape*>(shape_fixture->GetShape());
      size_t count = 10;
      std::vector<Vector2f> vertices(count);
      for (size_t i = 0; i < count; ++i) {
        float32 x =
            shape->m_p.x + shape->m_radius * cos(float32(2 * i) / count * M_PI);
        float32 y =
            shape->m_p.y + shape->m_radius * sin(float32(2 * i) / count * M_PI);
        vertices[i] = Vector2f(x, y) * kScaleUp;
      }
      new_node->AddGeometry(Polygon(vertices));
    }
  }  
}
*/

}  // namespace diagrammar
