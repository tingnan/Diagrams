// Copyright 2015 Native Client Authors.

#include <random>
#include <iostream>
#include <algorithm>

#include <json/json.h>
#include "physics/world.h"
#include "utility/world_parser.h"
#include "physics/physics_engine_liquidfun.h"

namespace diagrammar {

void World::_ConstructWorldFromDescriptor(const Json::Value& world) {
  // using the WorldDescptor to construct the initial world
  // here we will work on the json object
  Json::Value::const_iterator itr = world.begin();
  for (; itr != world.end(); ++itr) {
    if (itr.key().asString() == "transform") {
      coordinate_ = CoordinateFrame2D(ParseTransformation2D(*itr));
      // std::cout << mWorldTransformation.GetTransform().matrix() << std::endl;
    }
    if (itr.key().asString() == "children") {
      Json::Value::const_iterator child_itr = (*itr).begin();
      for (; child_itr != (*itr).end(); ++child_itr) {
        Node* ptr = AddNode(ParseNode(*child_itr));
      }
    }
  }
}

World::~World() {
  if (physics_engine_) {
    delete physics_engine_;
  }
}

void World::InitializeWorldDescription(const Json::Value& world) {
  _ConstructWorldFromDescriptor(world);
  world_state_ |= WorldStateFlag::kInitialized;
}

void World::InitializeWorldDescription(const char* file) {
  Json::Value WorldDescriptor = CreateJsonObject(file);
  _ConstructWorldFromDescriptor(WorldDescriptor);
  world_state_ |= WorldStateFlag::kInitialized;
}

void World::InitializePhysicsEngine(EngineType t) {
  physics_engine_ = new PhysicsEngineLiquidFun(*this);

  world_state_ |= WorldStateFlag::kEngineReady;
}

void World::InitializeTimer() { timer_.Initialize(); }

void World::Step() {
  // threaded support, handle the stepping in another thread
  // while the main thread continuously update the rendering context;
  if (world_state_ != WorldStateFlag::kRunning) return;
  int num_ticks = timer_.BeginNextFrame();
  assert(num_ticks >= 0);
  double before_step = timer_.now();
  for (int tick = num_ticks; tick > 0; --tick) {
    physics_engine_->Step();
  }

  double after_step = timer_.now();
  if (num_ticks > 0) {
    step_time_.push_back((after_step - before_step) / num_ticks);
    if (step_time_.size() > 200) {
      step_time_.pop_front();
    }
  }

  if (timer_.accumulated_ticks() % 200 == 0 && timer_.accumulated_ticks() > 0) {
    std::cerr << "200 counts: ";
    auto result = std::minmax_element(step_time_.begin(), step_time_.end());
    std::cerr << "min: " << *result.first << " ms ";
    std::cerr << "max: " << *result.second << " ms ";
    std::cerr << "mean: "
              << std::accumulate(step_time_.begin(), step_time_.end(), 0.0) /
                     step_time_.size()
              << " ms\n";
  }

  physics_engine_->SendDataToWorld();
}

void World::_GenerateID(Node* node_ptr) {
  std::random_device rd;
  std::mt19937 engine(rd());
  std::uniform_int_distribution<int> distro;
  if (node_table_.find(node_ptr->id()) == node_table_.end()) {
    node_table_.insert(std::make_pair(node_ptr->id(), node_ptr));
  } else {
    // try to generate a new id
    int id = distro(engine);
    while (node_table_.find(id) != node_table_.end()) {
      id = distro(engine);
    }
    // std::cout << id << std::endl;
    node_ptr->set_id(id);
    node_table_.insert(std::make_pair(id, node_ptr));
  }
}

Node* World::AddNode(Node obj) {
  nodes_.emplace_back(make_unique<Node>(std::move(obj)));
  _GenerateID(nodes_.back().get());
  return nodes_.back().get();
}

Node* World::GetNodeByIndex(int i) const {
  if (i < nodes_.size() && i >= 0) return nodes_[i].get();
  return nullptr;
}

Node* World::GetNodeByID(int id) const {
  if (node_table_.find(id) != node_table_.end()) {
    return node_table_.find(id)->second;
  }
  return nullptr;
}

size_t World::GetNumNodes() const { return nodes_.size(); }

float World::time_step() const { return timer_.tick_time(); }

float World::now() const { return timer_.now(); }

float World::simulation_time() const {
  return timer_.accumulated_ticks() * timer_.tick_time();
}

}  // namespace diagrammar
