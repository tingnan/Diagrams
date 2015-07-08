// Copyright 2015 Native Client authors

#ifndef RUNTIME_PHYSICS_WORLD_H_
#define RUNTIME_PHYSICS_WORLD_H_

#include <list>
#include <vector>
#include <unordered_map>

#include "utility/timer.h"
#include "utility/stl_memory.h"
#include "physics/node.h"

namespace Json {
class Value;
}

namespace diagrammar {

// This is a simulation world. A the wrapper class that communicates with a physics
// engine.
class World {
 public:
  enum EngineType { kDemo, kLiquidFun, kChipmunk2D, kBullet, kODE };
  World() = default;
  ~World();
  World(World&& other) = default;
  // not copyable
  World(const World& other) = delete;

  // initialize the world, the physics engine MUST be initialized after 
  // the world description is loaded
  void LoadWorld(const char* file);
  void InitializePhysicsEngine(EngineType t = EngineType::kLiquidFun);
  // called after physics engine is initialized
  void InitializeTimer();

  // put in the main loop
  void Step();

  // event handler, to be implemented with state charts
  void HandleEvent();

  float time_step() const;
  float now() const;
  float simulation_time() const;
  
  // Copy a node to the world and assign an id
  Node* AddNode(Node);
  Node* GetNodeByID(id_t) const;
  Node* GetNodeByIndex(size_t) const;
  size_t GetNumNodes();
  void RemoveNodeByID(id_t);
 private:
  // Clear everything in the world, reset the state to just created (not initialized).
  void Reset();

  // called by InitializeWorldDescription
  void ParseWorld(const Json::Value&);

  // world frame
  CoordinateFrame2D frame_;

  // quick access to node by unique id
  std::unordered_map<size_t, size_t> node_table_;
  std::vector<std::unique_ptr<Node> > nodes_;

  // Timer that sync the simulation with real time
  Timer timer_;
  std::list<double> step_time_;

  // the pointer to the actual physics engine used;
  // we will allow user to switch engine;
  class PhysicsEngine* physics_engine_;

  // node id counter
  int id_counter_ = 0;
  
};
}  // namespace diagrammar

#endif  // RUNTIME_PHYSICS_WORLD_H_
