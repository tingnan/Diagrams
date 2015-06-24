#ifndef PHYSICS_PHYSICS_ENGINE_LIQUIDFUN_
#define PHYSICS_PHYSICS_ENGINE_LIQUIDFUN_

// the demo physics engine
// can only do a few predefined stuff for now

#include "physics_engine.h"
class b2World;
class b2Body;
namespace diagrammar {
// this is a base class that is supposed to be overload
class PhysicsEngineLiquidFun : public PhysicsEngine {
 public:
  // this engine is special in that it will directly modify
  // the objects in the world
  // other physics engines only obtain immutable copies
  PhysicsEngineLiquidFun(World&);
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
  void AddNodeFromEngineToWorld(b2Body*);
  // we can either add polygons to the body
  void AddPolygonsToBody(const class ComplexShape2D&, b2Body*);
  // or add chains to the body
  void AddChainsToBody(const class ComplexShape2D&, b2Body*);

  class b2World* b2world_;
  int kVelocityIterations = 5;
  int kPositionIterations = 5;
  float kScaleUp = 10;
  float kScaleDown = 0.1f;
  float kDensity = 1.f;
  float kRestitution = 1.f;
  float kFriction = 0.1;
};
}

#endif
