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

// The class allows random access by index,
// and the address of every contained value is stable.
// TODO(tingnan), allow custom hash function
template<class Key, class Value>
class StableAddressRandomAccessMap {
 public:
  class iterator {

  };

  class const_ierator {

  };

  StableAddressRandomAccessMap() = default;
  StableAddressRandomAccessMap(const StableAddressRandomAccessMap&) = delete;
  size_t size() const;
  // TODO(tingnan) maybe implement iterator and find
  bool contains(const Key& key) const;
  // Value type must have a default constructor
  Value& operator[](const Key& key);
  // Access by index (random access)
  Value& get(size_t index) const;
  size_t erase(const Key& key);
 private:
  typedef std::pair<Key, std::unique_ptr<Value> > ValuePair;
  std::vector<ValuePair> container_;
  std::unordered_map<Key, size_t> lookup_table_;
};

template<class Key, class Value>
size_t StableAddressRandomAccessMap<Key, Value>::size() const {
  return container_.size();
}

template<class Key, class Value>
bool StableAddressRandomAccessMap<Key, Value>::contains(const Key& key) const {
  return lookup_table_.find(key) != lookup_table_.end();
}

template<class Key, class Value>
Value& StableAddressRandomAccessMap<Key, Value>::get(size_t index) const {
  return *container_[index].second.get();
}

template<class Key, class Value>
Value& StableAddressRandomAccessMap<Key, Value>::operator[](const Key& key) {
  if (contains(key))
    return *container_[lookup_table_[key]].second.get();
  // Create a new element with the key;
  container_.emplace_back(std::make_pair(key, make_unique<Value>()));
  lookup_table_[key] = container_.size() - 1;
  return *container_.back().second.get();
}

template<class Key, class Value>
size_t StableAddressRandomAccessMap<Key, Value>::erase(const Key& key) {
  if (contains(key)) {
    size_t idx = lookup_table_[key];
    if (idx != container_.size() - 1) {
      // Swap with last element
      container_[idx].swap(container_.back());
      lookup_table_[container_[idx].first] = idx;
    }
    // Erase the last element
    container_.pop_back();
    lookup_table_.erase(key);
    return 1;
  }
  return 0;
}

// TODO(tingnan) allow custom hash function
// This is a bijection map (one to one only)
template<class Key, class Value> 
class UnorderedBijectionMap{
 public:
  bool contains_key(const Key& key);
  bool contains_value(const Value& val);
  // If key/value is not contained, the returned reference will be invalid 
  // and may cause segfault
  Value& get_value(const Key& key);
  Key& get_key(const Value& val);
  void erase_by_key(const Key& key);
  void erase_by_value(const Value& val);
 private:
  std::unordered_map<Key, Value> key_value_map_;
  std::unordered_map<Key, Value> value_key_map_;
};

template<class Key, class Value> 
Value& UnorderedBijectionMap<Key, Value>::get_value(const Key& key) {
  return key_value_map_.find(key)->second;
}

template<class Key, class Value> 
Key& UnorderedBijectionMap<Key, Value>::get_key(const Value& val) {
  return value_key_map_.find(val)->second;
}


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
  Node* GetNodeByID(id_t);
  Node* GetNodeByIndex(size_t);
  size_t GetNumNodes();
  void RemoveNodeByExtID(id_t);
  void RemoveNodeByIntID(id_t);

 private:
  // Clear everything in the world.
  void Reset();

  // called by InitializeWorldDescription
  void ParseWorld(const Json::Value&);

  CoordinateFrame2D frame;

  // do the same for the joints
  // std::vector<std::unique_ptr<Node> > nodes_;
  // Quick access to node by unique id
  // std::unordered_map<id_t, size_t> node_table_;
  StableAddressRandomAccessMap<id_t, Node> node_map_;

  // Map external id to internal and vice versa
  std::unordered_map<id_t, id_t> idmap_ext_int_;
  std::unordered_map<id_t, id_t> idmap_int_ext_;

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
