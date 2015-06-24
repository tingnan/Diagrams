#ifndef PHYSICS_PHYSICS_ENGINE_DEMO_
#define PHYSICS_PHYSICS_ENGINE_DEMO_

// the demo physics engine
// can only do a few predefined stuff for now

#include "physics_engine.h"

namespace diagrammar {
// this is a base class that is supposed to be overload
class PhysicsEngineDemo : public PhysicsEngine {
 public:
  // this engine is special in that it will directly modify
  // the objects in the world
  // other physics engines only obtain immutable copies
  PhysicsEngineDemo(World&);
  ~PhysicsEngineDemo();
  // step the world
  void Step();
  void SendDataToWorld();
  // incase of a event, we also need apis to handle
  // to be implemented
  void HandleEvents(const Json::Value&);
};
}

#endif