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
  enum EngineType { kLiquidFun, kChipmunk2D, kBullet, kODE };
  World();
  ~World();
  World(World&& other) = default;
  // not copyable
  World(const World& other) = delete;

  // initialize the world, the physics engine MUST be initialized after 
  // the world description is loaded
  // TODO Change to Read and Start
  void Read(const Json::Value&);
  void Start(EngineType t = EngineType::kLiquidFun);

  // put in the main loop
  void Step();

  // Handle input event message
  void HandleMessage(const Json::Value&);

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

  // Clear everything in the world.
  void Reset();

  // called by InitializeWorldDescription
  void ParseWorld(const Json::Value&);

  // TODO world_frame_
  CoordinateFrame2D frame;

  std::vector<std::unique_ptr<Node> > nodes_;
  // Quick access to node by unique id
  std::unordered_map<id_t, size_t> node_table_;
  
  // Timer that sync the simulation with real time
  Timer timer_;
  
  // Circular buffer to calculate performance benchmark
  std::list<double> step_time_;

  // Pointer to the actual physics engine used;
  // we will allow user to switch engine;
  std::unique_ptr<class PhysicsEngine> physics_engine_;

  // node id counter
  int id_counter_ = 0;
  
};
}  // namespace diagrammar

#endif  // RUNTIME_PHYSICS_WORLD_H_
