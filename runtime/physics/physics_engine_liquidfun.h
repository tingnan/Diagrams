// Copyright 2015 Native Client Authors.

#ifndef RUNTIME_PHYSICS_PHYSICS_ENGINE_LIQUIDFUN_H_
#define RUNTIME_PHYSICS_PHYSICS_ENGINE_LIQUIDFUN_H_

#include <vector>
#include <unordered_map>

#include "physics/physics_engine.h"

class b2World;
class b2Body;
class b2Joint;

namespace diagrammar {
struct Node;
class Box2DJointDestructionListener;
class PhysicsEngineLiquidFun : public PhysicsEngine {
 public:
  explicit PhysicsEngineLiquidFun(float time_step);
  ~PhysicsEngineLiquidFun();

  void Step();
  void SendDataToWorld();

  void AddNode(Node* node);
  // The joint CANNOT be initialized, if the nodes it connect are not there yet.
  void AddJoint(Joint* joint);
  void RemoveNodeByID(id_t id);
  void RemoveJointByID(id_t id);

  void ApplyForceToNode(id_t, const Vector3f& force, const Vector3f& offset);
  void ApplyImpulseToNode(id_t, const Vector3f& impulse,
                          const Vector3f& offset);
  void ApplyTorqueToNode(id_t, const Vector3f& torque);
  void ApplyAngularImpulseToNode(id_t, const Vector3f& torque);

 private:
  std::unique_ptr<b2World> b2world_;
  std::unordered_map<id_t, b2Body*> body_table_;
  std::unordered_map<id_t, b2Joint*> joint_table_;
  std::unique_ptr<Box2DJointDestructionListener> joint_destruction_listener_;

  // Used internally for the LCP solver.
  // DONOT modify unless you know what they are.
  int velocity_iterations_ = 5;
  int position_iterations_ = 5;

  // The engine is running on a different length (force/torque) scale.
  const float kScaleUp = 40;
  const float kScaleDown = 0.025f;
};
}  // namespace diagrammar

#endif  // RUNTIME_PHYSICS_PHYSICS_ENGINE_LIQUIDFUN_H_
