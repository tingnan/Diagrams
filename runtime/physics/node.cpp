#include "node.h"
#include "utility/stl_memory.h"
namespace diagrammar {
Node::Node() {}

Node::Node(const Node& rhs)
    : coordinate_(rhs.coordinate_),
      properties_(rhs.properties_),
      id_(rhs.id_),
      collision_group_id_(rhs.collision_group_id_) {
  // deep copy
  collision_shapes_.reserve(rhs.collision_shapes_.size());
  for (const auto& ptr : rhs.collision_shapes_) {
    collision_shapes_.emplace_back(make_unique<ComplexShape2D>(*ptr));
  }
}

Node& Node::operator=(const Node& rhs) {
  assert(0);
  return *this;
}

Node::Node(ComplexShape2D geo) {
  collision_shapes_.emplace_back(make_unique<ComplexShape2D>(std::move(geo)));
}

// member accessible functions
const ComplexShape2D* Node::GetGeometry(unsigned i) const {
  return collision_shapes_[i].get();
}

ComplexShape2D* Node::GetGeometry(unsigned i) { return collision_shapes_[i].get(); }

void Node::AddGeometry(ComplexShape2D&& geo) {
  collision_shapes_.emplace_back(make_unique<ComplexShape2D>(geo));
}

int Node::GetID() const { return id_; }

void Node::SetID(int id) { id_ = id; }

int Node::GetCollisionGroupID() const { return collision_group_id_; }

void Node::SetCollisionGroupID(int id) { collision_group_id_ = id; }

float Node::GetRotationAngle() { return coordinate_.GetRotation().angle(); }

void Node::SetRotationAngle(float a) {
  coordinate_.SetRotation(Eigen::Rotation2Df(a));
}

void Node::SetRotationMatrix(const Eigen::Matrix2f& rotmat) {
  coordinate_.SetRotation(rotmat);
}

Eigen::Matrix2f Node::GetRotationMatrix() {
  return coordinate_.GetRotation().matrix();
}

void Node::Rotate(float a) { coordinate_.Rotate(Eigen::Rotation2Df(a)); }

Vec2f Node::GetPosition() { return coordinate_.GetTranslation(); }

void Node::SetPosition(const Vec2f& pos) { coordinate_.SetTranslation(pos); }

void Node::Translate(const Vec2f& ds) { coordinate_.Translate(ds); }
}
