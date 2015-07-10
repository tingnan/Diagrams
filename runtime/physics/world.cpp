// Copyright 2015 Native Client Authors.

#include <random>
#include <iostream>
#include <algorithm>

#include <json/json.h>
#include "physics/world.h"
#include "utility/world_parser.h"
#include "physics/physics_engine_liquidfun.h"

namespace diagrammar {

World::World() {
}

World::~World() {
}

void World::Reset() {
  node_table_.clear();
  frame = CoordinateFrame2D(Isometry2f::Identity());
  step_time_.clear();
  id_counter_ = 0;
  auto engine_ptr = physics_engine_.release();
  if (engine_ptr)
    delete engine_ptr;
}

void World::Read(const Json::Value& world) {
  Reset();
  ParseWorld(world);
}

void World::Start(EngineType engine_type) {
  auto engine_ptr = physics_engine_.release();
  if (engine_ptr)
    delete engine_ptr;
  
  // Now generate some sample particles
  if (true) {
    std::random_device rd;
    std::uniform_real_distribution<float> pos_distx(-250, 50.f);
    std::uniform_real_distribution<float> pos_disty(50.f, 100.f);
    std::uniform_real_distribution<float> vel_dist(-50.f, 50.f);
    std::default_random_engine generator(rd());

    // create a circle

    Path circle;
    const size_t num_vertices = 30;
    for (int i = 0; i < num_vertices; ++i) {
      circle.emplace_back(5 *
                          Vector2f(cos(float(2 * i) * M_PI / num_vertices),
                                   sin(float(2 * i) * M_PI / num_vertices)));
    }
    for (int i = 0; i < 1; ++i) {
      Polygon poly = Polygon(circle);
      poly.shape_type = OptimizedShapeType::kSphere2D;
      Node* node_ptr = AddNode(Node());
      node_ptr->polygons.emplace_back(poly);
      node_ptr->is_dynamic = true;
      node_ptr->frame.SetTranslation(
          Vector2f(pos_distx(generator), pos_disty(generator)));
      node_ptr->velocity = Vector2f(vel_dist(generator), vel_dist(generator));
    }
  }

  // TODO (add more physics engines)
  switch(engine_type) {
    case kLiquidFun:
      physics_engine_.reset(new PhysicsEngineLiquidFun(time_step()));
      break;
    default:
      physics_engine_.reset(new PhysicsEngineLiquidFun(time_step()));
      break;
  }

  for (auto itr = nodes_.begin(); itr != nodes_.end(); ++itr) {
    physics_engine_->AddNode(itr->get());
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

Node* World::AddNode(Node obj) {
  // The vector actually stores the pointer to the nodes
  // while the lookup table maps the unique id to the location in the vector
  nodes_.emplace_back(make_unique<Node>(std::move(obj)));
  auto node_ptr = nodes_.back().get();
  // Currently id_t is just an integer (increased counter)
  // We may decide later to recycle the ids, or purely assign the id from
  // outside
  id_counter_++;
  node_ptr->id = id_counter_;
  node_table_.insert(std::make_pair(id_counter_, nodes_.size() - 1));

  // now we also would like to add/remove the same node from the underlying
  // engine
  if (physics_engine_) {
    physics_engine_->AddNode(node_ptr);
  }

  return node_ptr;
}

void World::RemoveNodeByID(id_t id) {
  if (node_table_.find(id) != node_table_.end()) {
    size_t index = node_table_.find(id)->second;
    if (index != nodes_.size() - 1) {
      // Swap the back of nodes_ with current one
      nodes_[index].swap(nodes_.back());
      node_table_[nodes_[index]->id] = index;
    }
    node_table_.erase(id);
    nodes_.pop_back();

    // Now remove from the physics engine
    if (physics_engine_) {
      physics_engine_->RemoveNodeByID(id);
    }
  }
}

Node* World::GetNodeByID(id_t id) const {
  if (node_table_.find(id) != node_table_.end()) {
    return nodes_[node_table_.find(id)->second].get();
  }
  return nullptr;
}

Node* World::GetNodeByIndex(size_t index) const {
  assert(index < nodes_.size());
  return nodes_[index].get();
}

size_t World::GetNumNodes() { return nodes_.size(); }

float World::time_step() const { return timer_.tick_time(); }

float World::now() const { return timer_.now(); }

float World::simulation_time() const {
  return timer_.accumulated_ticks() * timer_.tick_time();
}

void World::ParseWorld(const Json::Value& world) {
  // using the WorldDescptor to construct the initial world
  // here we will work on the json object
  Json::Value::const_iterator itr = world.begin();
  for (; itr != world.end(); ++itr) {
    if (itr.key().asString() == "transform") {
      frame = CoordinateFrame2D(ParseTransformation2D(*itr));
      // std::cout << mWorldTransformation.GetTransform().matrix() << std::endl;
    }
    if (itr.key().asString() == "children") {
      Json::Value::const_iterator child_itr = (*itr).begin();
      for (; child_itr != (*itr).end(); ++child_itr) {
        AddNode(ParseNode(*child_itr));
      }
    }
  }
}

}  // namespace diagrammar
