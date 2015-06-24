// Copyright 2015 Native Client authors
#ifndef RUNTIME_PHYSICS_PHYSICS_ENGINE_LIQUIDFUN_H_
#define RUNTIME_PHYSICS_PHYSICS_ENGINE_LIQUIDFUN_H_

// the demo physics engine
// can only do a few predefined stuff for now

#include "physics/physics_engine.h"
class b2World;
class b2Body;
namespace diagrammar {
// this is a base class that is supposed to be overload
class PhysicsEngineLiquidFun : public PhysicsEngine {
 public:
  // this engine is special in that it will directly modify
  // the objects in the world
  // other physics engines only obtain immutable copies
  explicit PhysicsEngineLiquidFun(World&);
  ~PhysicsEngineLiquidFun();

  void SendDataToWorld();
  void Step();
  // incase of a event, we also need apis to handle
  // to be implemented
  void HandleEvents(const Json::Value&);

 private:
  // method to transfer node between physics engine and the world class
  // allow runtime adding things
  void AddNodeFromWorldToEngine(Node* ref);
  void AddNodeFromEngineToWorld(b2Body* body);
  // we can either add polygons to the body
  void AddPolygonsToBody(const class ComplexShape2D&, b2Body*);
  // or add chains to the body
  void AddChainsToBody(const class ComplexShape2D&, b2Body*);

  class b2World* b2world_;
  // a default set of constant
  int velocity_iterations_ = 5;
  int position_iterations_ = 5;
  const float kScaleUp = 10;
  const float kScaleDown = 0.1f;
  const float kDefaultDensity = 1.f;
  const float kDefaultRestitution = 1.f;
  const float kDefaultFriction = 0.1;
};
}  // namespace diagrammar

#endif  // RUNTIME_PHYSICS_PHYSICS_ENGINE_LIQUIDFUN_H_
