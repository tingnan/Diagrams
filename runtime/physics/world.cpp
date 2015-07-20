// Copyright 2015 Native Client Authors.

#include <json/json.h>

#include <random>
#include <iostream>
#include <algorithm>

#include "physics/world.h"
#include "utility/world_parser.h"
#include "physics/physics_engine_liquidfun.h"

namespace diagrammar {

World::World() {}

World::~World() {}

void World::Reset() {
  // node_table_.clear();
  frame = CoordinateFrame2D(Isometry2f::Identity());
  step_time_.clear();
  id_counter_ = 0;
  auto engine_ptr = physics_engine_.release();
  if (engine_ptr) delete engine_ptr;
}

void World::Read(const Json::Value& world) {
  Reset();
  ParseWorld(world);
}

void World::Start(EngineType engine_type) {
  auto engine_ptr = physics_engine_.release();
  if (engine_ptr) delete engine_ptr;

  // Now generate some sample particles
  if (true) {
    std::random_device rd;
    std::uniform_real_distribution<float> pos_distx(-250, 50.f);
    std::uniform_real_distribution<float> pos_disty(50.f, 100.f);
    std::uniform_real_distribution<float> vel_dist(-250.f, 250.f);
    std::default_random_engine generator(rd());

    // create a disk
    Path circle;
    const size_t num_vertices = 30;
    for (int i = 0; i < num_vertices; ++i) {
      circle.emplace_back(5 * Vector2f(cos((2.0 * i) * M_PI / num_vertices),
                                       sin((2.0 * i) * M_PI / num_vertices)));
    }
    for (int i = 0; i < 100; ++i) {
      Polygon poly = Polygon(circle);
      poly.shape_info["type"] = static_cast<int>(ShapeType::kDisk);
      poly.shape_info["radius"] = static_cast<float>(5.0);
      Node* node_ptr = AddNode(Node());
      node_ptr->polygons.emplace_back(poly);
      node_ptr->is_dynamic = true;
      node_ptr->material_info.restitution = 0.98;
      node_ptr->frame.SetTranslation(
          Vector2f(pos_distx(generator), pos_disty(generator)));
      node_ptr->velocity = Vector2f(vel_dist(generator), vel_dist(generator));
    }
  }

  // TODO(tingnan) add more physics engines
  switch (engine_type) {
    case kLiquidFun:
      physics_engine_.reset(new PhysicsEngineLiquidFun(time_step()));
      break;
    default:
      physics_engine_.reset(new PhysicsEngineLiquidFun(time_step()));
      break;
  }

  for (size_t index = 0; index < node_map_.size(); ++index) {
    physics_engine_->AddNode(node_map_.get(index).get());
  }

  timer_.Initialize();
}

void World::Step() {
  if (!physics_engine_) return;

  // The timer returns the proper number of steps the engine needs to execute
  // If the elapsed time is too long, only one step is returned.
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
    std::cout << "200 counts: ";
    auto result = std::minmax_element(step_time_.begin(), step_time_.end());
    std::cout << "min: " << *result.first << " ms ";
    std::cout << "max: " << *result.second << " ms ";
    std::cout << "mean: "
              << std::accumulate(step_time_.begin(), step_time_.end(), 0.0) /
                     step_time_.size()
              << " ms\n";
  }

  physics_engine_->SendDataToWorld();
}

Node* World::AddNode(Node tmp_node) {
  
  // nodes_.emplace_back(make_unique<Node>(std::move(tmp_node)));
  // auto node_ptr = nodes_.back().get();

  id_t ext_id = tmp_node.id;
  id_counter_++;
  // Move the content to the map
  node_map_[id_counter_] = make_unique<Node>(std::move(tmp_node));
  Node* node_ptr = node_map_[id_counter_].get();
  node_ptr->id = id_counter_;

  // Create the id mapping
  idmap_ext_int_[ext_id] = node_ptr->id;
  idmap_int_ext_[node_ptr->id] = ext_id;

  // Now we also would like to add the same node from the underlying
  // engine
  if (physics_engine_) {
    physics_engine_->AddNode(node_ptr);
  }

  return node_ptr;
}

void World::RemoveNodeByIntID(id_t id) {
  node_map_.erase(id);
  idmap_ext_int_.erase(idmap_int_ext_[id]);
  idmap_int_ext_.erase(id);

  if (physics_engine_) {
      physics_engine_->RemoveNodeByID(id);
  }
}


void World::RemoveNodeByExtID(id_t id) {
  RemoveNodeByIntID(idmap_ext_int_[id]);
}

Node* World::GetNodeByID(id_t id) {
  if (node_map_.contains(id)) {
    return node_map_[id].get();
  }
  return nullptr;
}

Node* World::GetNodeByIndex(size_t index) {
  assert(index < node_map_.size());
  return node_map_.get(index).get();
}

size_t World::GetNumNodes() { return node_map_.size(); }

float World::time_step() const { return timer_.tick_time(); }

float World::now() const { return timer_.now(); }

float World::simulation_time() const {
  return timer_.accumulated_ticks() * timer_.tick_time();
}

void World::ParseWorld(const Json::Value& world) {
  // Construct the initial world
  
  if (world.isMember("transform")) {
    frame = CoordinateFrame2D(ParseTransformation2D(world["transform"]));
  }

  // 
  const Json::Value& child_obj = world["children"];
  Json::Value::const_iterator child_itr = child_obj.begin();
  for (; child_itr != child_obj.end(); ++child_itr) {
    AddNode(ParseNode(*child_itr));
  }

  // can only parse the joint, after we know the children
  const Json::Value& joint_obj = world["joints"];
  Json::Value::const_iterator joint_itr = joint_obj.begin();
}

}  // namespace diagrammar
