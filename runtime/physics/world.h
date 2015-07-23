// Copyright 2015 Native Client authors

#ifndef RUNTIME_PHYSICS_WORLD_H_
#define RUNTIME_PHYSICS_WORLD_H_

#include <list>
#include <vector>
#include <unordered_map>

#include <boost/bimap.hpp>
#include <boost/bimap/unordered_set_of.hpp>
#include "utility/map.h"
#include "utility/timer.h"
#include "utility/stl_memory.h"
#include "physics/node.h"

namespace Json {
class Value;
}

namespace diagrammar {

// This is a simulation world. A the wrapper class that communicates with a
// physics
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
  void Read(const Json::Value&);
  void Start(EngineType t = EngineType::kLiquidFun);

  // put in the main loop
  void Step();

  // Handle input event message
  void HandleMessage(const Json::Value&) {}

  float time_step() const;
  float now() const;
  float simulation_time() const;

  // Copy a node to the world and assign an id
  Node* AddNode(Node);
  Node* GetNodeByExtID(id_t) const;
  Node* GetNodeByIntID(id_t) const;
  Node* GetNodeByIndex(size_t);
  size_t GetNumNodes();
  Node* RemoveNodeByExtID(id_t);
  Node* RemoveNodeByIntID(id_t);

  // For joints
  Joint* AddJoint(std::unique_ptr<Joint> base_ptr);
  Joint* GetJointByExtID(id_t) const;
  Joint* GetJointByIntID(id_t) const;
  Joint* GetJointByIndex(size_t);
  size_t GetNumJoints();
  Joint* RemoveJointByExtID(id_t);
  Joint* RemoveJointByIntID(id_t);

 private:
  struct ext_id {};
  struct int_id {};
  typedef boost::bimap<
      boost::bimaps::unordered_set_of<boost::bimaps::tagged<id_t, ext_id>>,
      boost::bimaps::unordered_set_of<boost::bimaps::tagged<id_t, int_id>>>
      IDMap;

  class IDPool {
   public:
    // Get a new, unused id;
    id_t GetID() { return id_counter_++; }
    // Do nothing for now, ideally will recycle unused id;
    void RecycleID(id_t freed_id){};
    void Reset() { id_counter_ = 0; }

   private:
    id_t id_counter_ = 0;
  };

  // Clear everything in the world.
  void Reset();

  // called by InitializeWorldDescription
  void ParseWorld(const Json::Value&);

  CoordinateFrame2D frame;

  // Quick access to node by unique id
  IndexedMap<id_t, std::unique_ptr<Node>> node_map_;
  // Map external id to internal and vice versa
  IDMap node_id_map_;
  // Do the same for joints
  IndexedMap<id_t, std::unique_ptr<Joint>> joint_map_;
  IDMap joint_id_map_;

  // Timer that sync the simulation with real time
  Timer timer_;

  // Circular buffer to calculate performance benchmark
  std::list<double> step_time_;

  // Pointer to the actual physics engine used;
  // we will allow user to switch engine;
  std::unique_ptr<class PhysicsEngine> physics_engine_;

  // node id counter
  IDPool id_pool_;
};

}  // namespace diagrammar

#endif  // RUNTIME_PHYSICS_WORLD_H_
