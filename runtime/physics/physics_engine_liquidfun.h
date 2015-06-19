#ifndef _DIAGRAMMAR_PHYSICSENGINE_LIQUIDFUN_
#define _DIAGRAMMAR_PHYSICSENGINE_LIQUIDFUN_

// the demo physics engine
// can only do a few predefined stuff for now

#include "physics_engine.h"
class b2World;
class b2Body;
namespace diagrammar {
// this is a base class that is supposed to be overload
class PhysicsEngineLiquidFun : public PhysicsEngine {
 private:
  class b2World* b2world_;
  int kVelocityIterations = 5;
  int kPositionIterations = 5;
  float kScaleUp = 10;
  float kScaleDown = 0.1f;
  float kDensity = 1.f;
  float kRestitution = 1.f;
  float kFriction = 0.1;

 private:
  //
  void _AddNodeToEngine(Node* ref);
  void _AddPolygonToBody(const class Geometry2D&, b2Body*);
  //
  void _AddShapeToWorld(b2Body*);
  void _AddChainToBody(const class Geometry2D&, b2Body*);

 public:
  // this engine is special in that it will directly modify
  // the objects in the world
  // other physics engines only obtain immutable copies
  PhysicsEngineLiquidFun(World&);
  ~PhysicsEngineLiquidFun();

  void SendDataToWorld();
  void Step();

 public:
  // incase of a event, we also need apis to handle
  // to be implemented
  void HandleEvents(const Json::Value&);
};
}

#endif
