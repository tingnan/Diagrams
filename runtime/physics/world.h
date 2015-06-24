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

// it is a simulation world. A the wrapper class that communicate with a physics engine
class World {
 public:
  enum EngineType { kDemo, kLiquidFun, kChipmunk2D, kBullet, kODE };
  World();
  ~World();
  World(World&& other) = default;
  // not copyable
  World(const World& other) = delete;
  
  // initialize the world structure with a description file
  void InitializeWorldDescription(const Json::Value& world);
  void InitializeWorldDescription(const char* file);

  // initialize a physics engine of user's choice
  // default is liquidfun/Box2D
  void InitializePhysicsEngine(EngineType t = EngineType::kLiquidFun);

  void InitializeTimer();
  // step the world!
  void Step();

  // event handler
  void HandleEvent();

  // general get and set function
  float time_step() const;
  float now() const;
  float simulation_time() const;
  // create an empty node
  Node* AddNode();
  // create a node from another
  Node* AddNode(Node);
  size_t GetNumNodes() const;
  Node* GetNodeByID(int id) const;
  Node* GetNodeByIndex(int index) const;

 private:
  // the state machine that controls the flow of simulation
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

  // using the Json object to construct an initial world
  void _ConstructWorldFromDescriptor(const Json::Value&);
  // assign a unique ID to the Node, if not already has
  void _GenerateID(Node*);
  
  // world frame
  CoordinateFrame2D coordinate_;

  // the vector that contains all the custom physics objects in the world
  // the vector includes objects created using json file, or later from JS
  // the vector DO NOT contain pinballs, which should be efficiently handled
  // using ObjectBatch class;
  std::vector<std::unique_ptr<Node> > nodes_;
  std::unordered_map<int, Node*> node_table_;
  // Timer that controls the simulation
  Timer timer_;
  std::list<double> step_time_;
  // the pointer to the actual physics engine used;
  // we will allow user to switch engine;
  class PhysicsEngine* physics_engine_;

  int world_state_ = WorldStateFlag::kCreated;
};
}  // namespace diagrammar

#endif  // RUNTIME_PHYSICS_WORLD_H_
