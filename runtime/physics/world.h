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

// proposed.
// can create a snapshop of the world
// can be used to reset the system
// support roll back (std::queue<snapshot>, which has fixed length)
// will not implement this untill we have pretty much
// all the modules functioning
// the snapshot updating may be in a separate thread
class Snapshot {};

// it is a simulation world. A the wrapper class that communicate with a physics
// engine
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
  void InitializeWorldDescription(const Json::Value& world);
  void InitializeWorldDescription(const char* file);
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
  size_t GetNumNodes() const;
  Node* GetNodeByID(int id) const;
  // get a ptr to a node by index (only for interating purpose) 
  Node* GetNodeByIndex(int index) const;

 private:

  // the state machine that controls the flow of simulation
  // will be replaced with boost state charts
  struct WorldStateFlag {
    static const int kCreated =
        0x0;  // it is just freshly baked and has nothing
    static const int kInitialized =
        0x1;  // now we at least initialized the world with sth
    static const int kEngineReady =
        0x2;  // we have enabled the physics engine to run the world
    static const int kRunning = 0x3;  // Initialized | EngineReady bit flag set!
    static const int kHalt = 0x4;
  };
  
  // called by InitializeWorldDescription
  void _ConstructWorldFromDescriptor(const Json::Value&);

  // assign a unique ID to the Node, if not already has
  void _GenerateID(Node*);

  // world frame
  CoordinateFrame2D frame_;

  std::vector<std::unique_ptr<Node> > nodes_;
  // quick access to node by unique id
  std::unordered_map<int, Node*> node_table_;

  // Timer that sync the simulation with real time
  Timer timer_;
  std::list<double> step_time_;

  // the pointer to the actual physics engine used;
  // we will allow user to switch engine;
  class PhysicsEngine* physics_engine_;

  // TODO(tingnan), replace with state machine
  int world_state_ = WorldStateFlag::kCreated;
};
}  // namespace diagrammar

#endif  // RUNTIME_PHYSICS_WORLD_H_
