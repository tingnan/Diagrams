// Copyright 2015 Native Client authors

#ifndef RUNTIME_PHYSICS_PHYSICS_ENGINE_H_
#define RUNTIME_PHYSICS_PHYSICS_ENGINE_H_

#include <vector>

#include <physics/node.h>

namespace Json {
class Value;
}  // namespace Json

namespace diagrammar {

// this is a base class that is supposed to be overload
// allow for different physics engines
class PhysicsEngine {
 public:
  
  explicit PhysicsEngine(float time_step) : time_step_(time_step) {}
  PhysicsEngine(const PhysicsEngine&) = delete;
  PhysicsEngine(PhysicsEngine&&) = default;
  virtual ~PhysicsEngine() = default;
  
  // step the internal world
  virtual void Step() = 0;
  // update the world
  virtual void SendDataToWorld() = 0;

  virtual void AddNode(Node*) = 0;

  // Remove a node using id
  virtual void RemoveNodeByID(id_t) = 0;

  // Add a constraint
  virtual void AddJoint(Joint*) = 0;

 protected:
  float time_step_;
};

}  // namespace diagrammar

#endif  // RUNTIME_PHYSICS_PHYSICS_ENGINE_H_
