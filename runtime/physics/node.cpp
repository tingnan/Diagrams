// Copyright 2015 Native Client authors.

#include "physics/node.h"
#include "utility/stl_memory.h"

namespace diagrammar {

Node::Node(const Node& rhs) :  
  frame_(rhs.frame_),
  properties_(rhs.properties_),
  id_(rhs.id_),
  collision_group_id_(rhs.collision_group_id_) {
  // deep copy content of unique_ptr
  polygons_.reserve(rhs.polygons_.size());
  for (const auto& geo_ptr : rhs.polygons_) {
    polygons_.emplace_back(make_unique<Polygon>(*geo_ptr));
  }

  paths_.reserve(rhs.paths_.size());
  for (const auto& geo_ptr : rhs.paths_) {
    paths_.emplace_back(make_unique<Path>(*geo_ptr));
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
  swap(frame_, rhs.frame_);
  swap(polygons_, rhs.polygons_);
  swap(paths_, rhs.paths_);
}

Polygon* Node::GetPolygon(unsigned i) const {
  return polygons_[i].get();
}

Path* Node::GetPath(unsigned i) const {
  return paths_[i].get();
}

void Node::AddGeometry(Polygon geo) {
  polygons_.emplace_back(make_unique<Polygon>(std::move(geo)));
}

void Node::AddGeometry(Path geo) {
  paths_.emplace_back(make_unique<Path>(std::move(geo)));
}

float Node::GetRotationAngle() const { return frame_.GetRotationAngle(); }

void Node::SetRotationAngle(float a) { frame_.SetRotation(Rotation2f(a)); }

void Node::SetRotationMatrix(Matrix2f rotmat) {
  frame_.SetRotation(rotmat);
}

Matrix2f Node::GetRotationMatrix() const {
  return frame_.GetRotationMatrix();
}

void Node::Rotate(float a) { frame_.Rotate(Rotation2f(a)); }

Vector2f Node::GetPosition() const { return frame_.GetTranslation(); }

void Node::SetPosition(const Vector2f& pos) { frame_.SetTranslation(pos); }

void Node::Translate(const Vector2f& ds) { frame_.Translate(ds); }


Vector2f Node::GetVelocity() const {
  return frame_.GetVelocity();
}

float Node::GetAngularVelocity() const {
  return frame_.GetAngularVelocity();
}

void Node::SetVelocity(Vector2f v) {
  frame_.SetVelocity(v);
}

void Node::SetAngularVelocity(float omega) {
  frame_.SetAngularVelocity(omega);
}

}  // namespace diagrammar
