#ifndef PHYSICS_PHYSICS_ENGINE_
#define PHYSICS_PHYSICS_ENGINE_

#include <vector>

namespace Json {
class Value;
}

namespace diagrammar {
// this is a base class that is supposed to be overload
class Node;
class PhysicsEngine {
 public:
  PhysicsEngine(class World& world) : world_(world) {}
  PhysicsEngine(PhysicsEngine&) = delete;
  virtual ~PhysicsEngine() {}
  // step the internal world
  virtual void Step() = 0;
  // update the world
  virtual void SendDataToWorld() = 0;
  // incase of a event, we also need apis to handle
  // to be implemented
  virtual void HandleEvents(const Json::Value&) = 0;
 protected:
  // maintain a reference layer
  class World& world_;
};
}

#endif