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

PhysicsEngineLiquidFun::PhysicsEngineLiquidFun(World& world)
    : PhysicsEngine(world) {
  // we now try to also create a box2d world
  b2Vec2 gravity(0.f, -4.9f);
  b2world_ = new b2World(gravity);
  // now we initialize the world using existing objects
  for (size_t i = 0; i < world_.GetNumNodes(); ++i) {
    AddNodeFromWorldToEngine(world_.GetNodeByIndex(i));
  }

  {
    std::random_device rd;
    std::uniform_real_distribution<float> pos_distx(-25.f, 5.f);
    std::uniform_real_distribution<float> pos_disty(5.f, 10.f);
    std::uniform_real_distribution<float> vel_dist(-5.f, 5.f);
    std::default_random_engine generator(rd());
    for (size_t i = 0; i < 250; ++i) {
      b2BodyDef particle_def;
      float x = pos_distx(generator);
      float y = pos_disty(generator);
      particle_def.position.Set(x, y);
      particle_def.type = b2_dynamicBody;
      b2Body* particle = b2world_->CreateBody(&particle_def);

      float vx = vel_dist(generator);
      float vy = vel_dist(generator);

      particle->SetLinearVelocity(b2Vec2(vx, vy));
      b2CircleShape particle_shape;
      particle_shape.m_radius = 0.4f;
      // b2PolygonShape particle_shape;
      // particle_shape.SetAsBox(0.2f, 0.2f);

      b2FixtureDef particle_fixture;
      particle_fixture.shape = &particle_shape;
      particle_fixture.density = kDefaultDensity;
      particle_fixture.friction = kDefaultFriction;
      particle_fixture.restitution = kDefaultRestitution;

      particle->CreateFixture(&particle_fixture);
      AddNodeFromEngineToWorld(particle);
    }
  }
}

void PhysicsEngineLiquidFun::AddNodeFromEngineToWorld(b2Body* body) {
  Node* new_node = world_.AddNode(Node());
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

void PhysicsEngineLiquidFun::AddNodeFromWorldToEngine(Node* ref) {
  b2BodyDef def;
  Vector2f pos = ref->GetPosition();
  def.position.Set(pos(0) * kScaleDown, pos(1) * kScaleDown);
  def.angle = ref->GetRotationAngle();
  b2Body* body = b2world_->CreateBody(&def);
  // the body will keep a pointer to the node
  body->SetUserData(ref);

  // create a set of triangles, for each geometry the node has
  for (unsigned count = 0; count < ref->GetNumPolygon(); ++count) {
    Polygon* poly = ref->GetPolygon(count);
    AddTrianglesToBody(TriangulatePolygon(*poly), body);
  }

  for (unsigned count = 0; count < ref->GetNumPolyline(); ++count) {
    Polyline* line = ref->GetPolyline(count);
    AddTrianglesToBody(TriangulatePolyline(*line, 1.5), body);
  }

  // AddRevoluteJointToWorld(b2world_, body);
}

PhysicsEngineLiquidFun::~PhysicsEngineLiquidFun() {}

void PhysicsEngineLiquidFun::Step() {
  b2world_->Step(world_.time_step(), velocity_iterations_,
                 position_iterations_);
}

void PhysicsEngineLiquidFun::SendDataToWorld() {
  for (b2Body* b = b2world_->GetBodyList(); b; b = b->GetNext()) {
    Node* obj = reinterpret_cast<Node*>(b->GetUserData());
    if (obj) {
      Vector2f translation(b->GetPosition().x * kScaleUp,
                           b->GetPosition().y * kScaleUp);
      obj->SetPosition(translation);
      obj->SetRotationAngle(b->GetAngle());
    }
  }
}

void PhysicsEngineLiquidFun::HandleEvents(const Json::Value&) {}
}  // namespace diagrammar
