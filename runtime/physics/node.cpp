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

int Node::GetID() const { return id_; }

void Node::SetID(int id) { id_ = id; }

int Node::GetCollisionGroupID() const { return collision_group_id_; }

void Node::SetCollisionGroupID(int id) { collision_group_id_ = id; }

float Node::GetRotationAngle() { return coordinate_.GetRotation().angle(); }

void Node::SetRotationAngle(float a) { coordinate_.SetRotation(Rotation2f(a)); }

void Node::SetRotationMatrix(const Matrix2f& rotmat) {
  coordinate_.SetRotation(rotmat);
}

Matrix2f Node::GetRotationMatrix() {
  return coordinate_.GetRotation().matrix();
}

void Node::Rotate(float a) { coordinate_.Rotate(Rotation2f(a)); }

Vector2f Node::GetPosition() { return coordinate_.GetTranslation(); }

void Node::SetPosition(const Vector2f& pos) { coordinate_.SetTranslation(pos); }

void Node::Translate(const Vector2f& ds) { coordinate_.Translate(ds); }
}  // namespace diagrammar
