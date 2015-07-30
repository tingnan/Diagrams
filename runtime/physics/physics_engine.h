// Copyright 2015 Native Client authors

#ifndef RUNTIME_PHYSICS_PHYSICS_ENGINE_H_
#define RUNTIME_PHYSICS_PHYSICS_ENGINE_H_

#include <vector>

#include "physics/node.h"

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
  virtual void AddNode(Node* node) = 0;
  // Remove a node using id
  virtual void RemoveNodeByID(id_t) = 0;
  // Add a constraint
  virtual void AddJoint(Joint* joint) = 0;
  virtual void RemoveJointByID(id_t) = 0;
  virtual void ApplyForceToNode(id_t, const Vector3f& force,
                                const Vector3f& offset) = 0;
  virtual void ApplyImpulseToNode(id_t, const Vector3f& impulse,
                                  const Vector3f& offset) = 0;
  virtual void ApplyTorqueToNode(id_t, const Vector3f& torque) = 0;
  virtual void ApplyAngularImpulseToNode(id_t, const Vector3f& torque) = 0;

 protected:
  float time_step_;
};

}  // namespace diagrammar

#endif  // RUNTIME_PHYSICS_PHYSICS_ENGINE_H_
