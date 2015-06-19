#include "Box2D/Box2D.h"
#include "physics_engine_liquidfun.h"
#include "world.h"
#include "geometry/triangulate.h"
#include <random>
#include <iostream>

namespace {


b2Body* ground = nullptr;
void AddRevoluteJointToWorld(b2World* world, b2Body* body1) {
  b2RevoluteJointDef pin_def;
  pin_def.bodyA = body1;
  pin_def.bodyB = ground;
  pin_def.collideConnected = false;
  pin_def.localAnchorA.SetZero();
  b2Vec2 pos = body1->GetPosition();
  pin_def.localAnchorB.Set(pos.x, pos.y);
  world->CreateJoint(&pin_def);
}

}

namespace diagrammar {

PhysicsEngineLiquidFun::PhysicsEngineLiquidFun(World& world)
    : PhysicsEngine(world) {
  // we now try to also create a box2d world
  b2Vec2 gravity(0.f, -4.9f);
  b2world_ = new b2World(gravity);
  b2BodyDef ground_def;
  ground = b2world_->CreateBody(&ground_def);
  // now we initialize the world using existing objects
  for (size_t i = 0; i < world_.GetNumNodes(); ++i) {
    _AddNodeToEngine(world_.GetNodeByIndex(i));
  }

  if (true) {
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
      particle_fixture.density = kDensity;
      particle_fixture.friction = kFriction;
      particle_fixture.restitution = kRestitution;

      particle->CreateFixture(&particle_fixture);
      _AddShapeToWorld(particle);
    }
  }
}

void PhysicsEngineLiquidFun::_AddShapeToWorld(b2Body* body) {
  std::vector<Geometry2D> geo_list;
  for (b2Fixture* shape_fixture = body->GetFixtureList(); shape_fixture; shape_fixture = shape_fixture->GetNext()) {
    b2Shape::Type shape_type = shape_fixture->GetType();
    if (shape_type == b2Shape::e_polygon) {
      b2PolygonShape* shape = (b2PolygonShape*)shape_fixture->GetShape();
      size_t count = shape->GetVertexCount();
      std::vector<Vec2f> vertices(count);
      for (size_t i = 0; i < count; ++i) {
        b2Vec2 tmp = shape->GetVertex(i);
        vertices[i] = Vec2f(tmp.x, tmp.y) * kScaleUp;
      }
      geo_list.emplace_back(Geometry2D(vertices));
    }
    if (shape_type == b2Shape::e_circle) {
      b2CircleShape* shape = (b2CircleShape*)shape_fixture->GetShape();
      size_t count = 10;
      std::vector<Vec2f> vertices(count);
      for (size_t i = 0; i < count; ++i) {
        float32 x = shape->m_p.x + shape->m_radius * cos(float32(2 * i) / count * M_PI);
        float32 y = shape->m_p.y + shape->m_radius * sin(float32(2 * i) / count * M_PI);
        vertices[i] = Vec2f(x, y) * kScaleUp;
      }
      geo_list.emplace_back(Geometry2D(vertices));
    }
  }
  Node* tmp = world_.AddNode();
  for (auto& geo : geo_list) {
    tmp->AddGeometry(std::move(geo));
  }
  body->SetUserData(tmp);
}

void PhysicsEngineLiquidFun::_AddChainToBody(const Geometry2D& geo, b2Body* b) {
  const std::vector<Vec2f>& pts = geo.GetPath();
  std::vector<b2Vec2> vertices(pts.size());
  for (size_t i = 0; i < vertices.size(); ++i) {
    vertices[i].Set(pts[i](0) * kScaleDown,
                    pts[i](1) * kScaleDown);
  }
  b2ChainShape chain;
  if (geo.IsPathClosed()) {
    chain.CreateLoop(vertices.data(), vertices.size());
  } else {
    chain.CreateChain(vertices.data(), vertices.size());
  }
  b->CreateFixture(&chain, 1);
}

void PhysicsEngineLiquidFun::_AddPolygonToBody(const Geometry2D& geo,
                                               b2Body* b) {
  std::vector<Triangle2D> pieces = geo.Triangulate();
  for (size_t i = 0; i < pieces.size(); ++i) {
    b2Vec2 vertices[3];
    vertices[0].Set(pieces[i].p0(0) * kScaleDown, pieces[i].p0(1) * kScaleDown);
    vertices[1].Set(pieces[i].p1(0) * kScaleDown, pieces[i].p1(1) * kScaleDown);
    vertices[2].Set(pieces[i].p2(0) * kScaleDown, pieces[i].p2(1) * kScaleDown);
    b2PolygonShape polygon;
    polygon.Set(vertices, 3);

    b2FixtureDef polyfixture;
    polyfixture.shape = &polygon;
    polyfixture.density = kDensity;
    polyfixture.friction = kFriction;
    polyfixture.restitution = kRestitution;
    b->CreateFixture(&polyfixture);
  }
}

void PhysicsEngineLiquidFun::_AddNodeToEngine(Node* ref) {
  b2BodyDef def;
  Vec2f pos = ref->GetPosition();
  def.position.Set(pos(0) * kScaleDown, pos(1) * kScaleDown);
  def.angle = ref->GetRotationAngle();
  def.type = b2_dynamicBody;
  b2Body* body = b2world_->CreateBody(&def);
  // the body will keep a pointer to the objects
  body->SetUserData(ref);
  // body->SetLinearVelocity(b2Vec2(-40, -20));
  // body->SetAngularVelocity(3.14);

  // create a set of triangles, for each geometry the node has
  for (unsigned count = 0; count < ref->GetGeometryCount(); ++count) {
    _AddPolygonToBody(*ref->GetGeometry(count), body);
  }
  AddRevoluteJointToWorld(b2world_, body);
}

PhysicsEngineLiquidFun::~PhysicsEngineLiquidFun() {}

void PhysicsEngineLiquidFun::Step() {
  b2world_->Step(world_.time_step(), kVelocityIterations,
                 kPositionIterations);
}

void PhysicsEngineLiquidFun::SendDataToWorld() {
  for (b2Body* b = b2world_->GetBodyList(); b; b = b->GetNext()) {
    Node* obj = (Node*)b->GetUserData();
    if (obj) {
      Vec2f translation(b->GetPosition().x * kScaleUp,
                        b->GetPosition().y * kScaleUp);
      obj->SetPosition(translation);
      obj->SetRotationAngle(b->GetAngle());
    }
  }
}

void PhysicsEngineLiquidFun::HandleEvents(const Json::Value&) {}
}
