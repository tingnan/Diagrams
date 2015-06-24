// Copyright 2015 Native Client authors
#include "physics/node.h"
#include "utility/stl_memory.h"

namespace diagrammar {
Node::Node() {}

Node::Node(ComplexShape2D geo) {
  collision_shapes_.emplace_back(make_unique<ComplexShape2D>(std::move(geo)));
}

// member accessible functions
const ComplexShape2D* Node::GetGeometry(unsigned i) const {
  return collision_shapes_[i].get();
}

ComplexShape2D* Node::GetGeometry(unsigned i) {
  return collision_shapes_[i].get();
}

void Node::AddGeometry(ComplexShape2D geo) {
  collision_shapes_.emplace_back(make_unique<ComplexShape2D>(std::move(geo)));
}

int Node::id() const { return id_; }

void Node::set_id(int id) { id_ = id; }

int Node::collision_group_id() const { return collision_group_id_; }

void Node::set_collision_group_id(int id) { collision_group_id_ = id; }

float Node::GetRotationAngle() const { return coordinate_.GetRotationAngle(); }

void Node::SetRotationAngle(float a) { coordinate_.SetRotation(Rotation2f(a)); }

void Node::SetRotationMatrix(const Matrix2f& rotmat) {
  coordinate_.SetRotation(rotmat);
}

Matrix2f Node::GetRotationMatrix() const {
  return coordinate_.GetRotationMatrix();
}

void Node::Rotate(float a) { coordinate_.Rotate(Rotation2f(a)); }

Vector2f Node::GetPosition() const { return coordinate_.GetTranslation(); }

void Node::SetPosition(const Vector2f& pos) { coordinate_.SetTranslation(pos); }

void Node::Translate(const Vector2f& ds) { coordinate_.Translate(ds); }
}  // namespace diagrammar
