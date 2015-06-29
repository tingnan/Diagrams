// Copyright 2015 Native Client authors.

#include "physics/node.h"
#include "utility/stl_memory.h"

namespace diagrammar {

Node::Node(const Node& rhs) :  
  coordinate_(rhs.coordinate_),
  properties_(rhs.properties_),
  id_(rhs.id_),
  collision_group_id_(rhs.collision_group_id_) {
  // deep copy content of unique_ptr
  collision_shapes_.reserve(rhs.collision_shapes_.size());
  for (const auto& geo_ptr : rhs.collision_shapes_) {
    collision_shapes_.emplace_back(make_unique<ComplexPolygon>(*geo_ptr));
  }
}

Node& Node::operator = (Node rhs) {
  swap(rhs);
  return *this;
}

void Node::swap(Node& rhs) {
  using std::swap;
  swap(collision_group_id_, rhs.collision_group_id_);
  swap(id_, rhs.id_);
  swap(properties_, rhs.properties_);
  swap(coordinate_, rhs.coordinate_);
  swap(collision_shapes_, rhs.collision_shapes_);
}

Node::Node(const ComplexPolygon& geo) {
  collision_shapes_.emplace_back(make_unique<ComplexPolygon>(geo));
}

Node::Node(const std::vector<ComplexPolygon>& geo_list) {
  for (const auto& geo : geo_list) {
    collision_shapes_.emplace_back(make_unique<ComplexPolygon>(geo));
  }
}

// member accessible functions
const ComplexPolygon* Node::GetGeometry(unsigned i) const {
  return collision_shapes_[i].get();
}

ComplexPolygon* Node::GetGeometry(unsigned i) {
  return collision_shapes_[i].get();
}

void Node::AddGeometry(ComplexPolygon geo) {
  collision_shapes_.emplace_back(make_unique<ComplexPolygon>(std::move(geo)));
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
