// Copyright 2015 Native Client Authors.

#ifndef RUNTIME_PHYSICS_PHYSICS_ENGINE_BULLET_H_
#define RUNTIME_PHYSICS_PHYSICS_ENGINE_BULLET_H_

#include <vector>
#include <unordered_map>
#include <memory>

#include "physics/physics_engine.h"

// Bullet specific types
class btDefaultCollisionConfiguration;
class btCollisionDispatcher;
class btBroadphaseInterface;
class btSequentialImpulseConstraintSolver;
class btDiscreteDynamicsWorld;
class btCollisionShape;
class btRigidBody;
struct btDefaultMotionState;
class btTypedConstraint;

namespace diagrammar {
struct Node;
// In bullet the rigid body does not own the collision shapes (e.g. allow shared
// collision shapes between rigid bodies). Here we do not track any sharing.
struct btRigidBodyResource {
  std::unique_ptr<btRigidBody> body;
  std::unique_ptr<btDefaultMotionState> motion_state;
  // The shape associated with the rigid body
  std::unique_ptr<btCollisionShape> collision_shape;
  // In case the collision_shape is compound, we store the basic shapes here
  std::vector<std::unique_ptr<btCollisionShape>> child_shapes;
};

class PhysicsEngineBullet : public PhysicsEngine {
 public:
  explicit PhysicsEngineBullet(float time_step);
  ~PhysicsEngineBullet();

  void Step();
  void SendDataToWorld();

  void AddNode(Node* node);
  // The joint CANNOT be initialized, if the nodes it connect are not there yet.
  void AddJoint(Joint* joint);
  void RemoveNodeByID(id_t id);
  void RemoveJointByID(id_t id);

  void ApplyForceToNode(id_t, const Vector2f& force, const Vector2f& offset);
  void ApplyImpulseToNode(id_t, const Vector2f& impulse,
                          const Vector2f& offset);
  void ApplyTorqueToNode(id_t, float torque);
  void ApplyAngularImpulseToNode(id_t, float torque);

 private:
  // Bullet world specs params.
  std::unique_ptr<btDefaultCollisionConfiguration> bt_collision_config_;
  std::unique_ptr<btCollisionDispatcher> bt_collision_dispatcher_;
  std::unique_ptr<btBroadphaseInterface> bt_broad_phase_interface_;
  std::unique_ptr<btSequentialImpulseConstraintSolver> bt_solver_;
  std::unique_ptr<btDiscreteDynamicsWorld> btworld_;

  // Resource management.
  std::unordered_map<id_t, btRigidBodyResource> body_table_;
  btRigidBodyResource ground_plane_;
  std::unordered_map<btRigidBody*, Node*> body_node_table_;
  // joints
  std::unordered_map<id_t, std::unique_ptr<btTypedConstraint>> joint_table_;

  // The engine is running on a different length (force/torque) scale.
  const float kScaleUp = 40;
  const float kScaleDown = 0.025f;
  // All object will be extended in z-direction in [-Height/2, Height/2].
  const float kDepth = 10.0;
};

}  // namespace diagrammar

#endif  // RUNTIME_PHYSICS_PHYSICS_ENGINE_BULLET_H_
