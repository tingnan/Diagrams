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

  polylines_.reserve(rhs.polylines_.size());
  for (const auto& geo_ptr : rhs.polylines_) {
    polylines_.emplace_back(make_unique<Polyline>(*geo_ptr));
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
  swap(polylines_, rhs.polylines_);
}

Polygon* Node::GetPolygon(unsigned i) const {
  return polygons_[i].get();
}

Polyline* Node::GetPolyline(unsigned i) const {
  return polylines_[i].get();
}

void Node::AddGeometry(Polygon geo) {
  polygons_.emplace_back(make_unique<Polygon>(std::move(geo)));
}

void Node::AddGeometry(Polyline geo) {
  polylines_.emplace_back(make_unique<Polyline>(std::move(geo)));
}


int Node::id() const { return id_; }

void Node::set_id(int id) { id_ = id; }

int Node::collision_group_id() const { return collision_group_id_; }

void Node::set_collision_group_id(int id) { collision_group_id_ = id; }

float Node::GetRotationAngle() const { return frame_.GetRotationAngle(); }

void Node::SetRotationAngle(float a) { frame_.SetRotation(Rotation2f(a)); }

void Node::SetRotationMatrix(const Matrix2f& rotmat) {
  frame_.SetRotation(rotmat);
}

Matrix2f Node::GetRotationMatrix() const {
  return frame_.GetRotationMatrix();
}

void Node::Rotate(float a) { frame_.Rotate(Rotation2f(a)); }

Vector2f Node::GetPosition() const { return frame_.GetTranslation(); }

void Node::SetPosition(const Vector2f& pos) { frame_.SetTranslation(pos); }

void Node::Translate(const Vector2f& ds) { frame_.Translate(ds); }
}  // namespace diagrammar
