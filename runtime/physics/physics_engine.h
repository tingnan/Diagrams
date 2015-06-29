// Copyright 2015 Native Client authors

#ifndef RUNTIME_PHYSICS_PHYSICS_ENGINE_H_
#define RUNTIME_PHYSICS_PHYSICS_ENGINE_H_

#include <vector>

namespace Json {
class Value;
}  // namespace Json

namespace diagrammar {

class Node;
// this is a base class that is supposed to be overload
// allow for different physics engines
class PhysicsEngine {
 public:
  explicit PhysicsEngine(class World& world) : world_(world) {}
  PhysicsEngine(const PhysicsEngine&) = delete;
  PhysicsEngine(PhysicsEngine&&) = default;
  virtual ~PhysicsEngine() = default;
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
}  // namespace diagrammar

#endif  // RUNTIME_PHYSICS_PHYSICS_ENGINE_H_
