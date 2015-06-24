// Copyright 2015 Native Client authors
#include <json/json.h>
#include "physics_engine_demo.h"
#include "world.h"

namespace diagrammar {
// this engine is special in that it will directly modify
// the objects in the world
// other physics engines only obtain immutable copies
PhysicsEngineDemo::PhysicsEngineDemo(World& world) : PhysicsEngine(world) {}

PhysicsEngineDemo::~PhysicsEngineDemo() {}

// not yet implemented
// virtual void AddConstraints() = 0;
// step the world
void PhysicsEngineDemo::Step() {
  double systime = world_.now();
  if (systime < 3) {
    for (size_t i = 0; i < world_.GetNumNodes(); ++i) {
      // update each of the object, now only using a funny code
      world_.GetNodeByIndex(i)->Rotate(world_.time_step() * 5);
    }
    systime += world_.time_step();
    return;
  }
}

void PhysicsEngineDemo::SendDataToWorld() {}

void PhysicsEngineDemo::HandleEvents(const Json::Value&) {}
}  // namespace diagrammar
