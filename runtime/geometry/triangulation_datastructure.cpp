// Copyright 2015 Native Client Authors.
#include "geometry/triangulation_datastructure.h"
#include "utility/stl_memory.h"

namespace diagrammar {

MeshDataStructure::Simplex2D::Simplex2D(Vertex2D* a, Vertex2D* b, Vertex2D* c,
                                        Simplex2D* fa, Simplex2D* fb,
                                        Simplex2D* fc)
    : vertices_{a, b, c}, faces_{fa, fb, fc} {}

MeshDataStructure::Vertex2D* MeshDataStructure::Simplex2D::GetVertex(
    unsigned i) const {
  unsigned index = i % 3;
  return vertices_[index];
}

int MeshDataStructure::Simplex2D::GetVertexIndex(Vertex2D* v) const {
  for (unsigned i = 0; i < 2; ++i) {
    if (v == vertices_[i]) return i;
  }
  return -1;
}

MeshDataStructure::Simplex2D* MeshDataStructure::Simplex2D::GetNeighbor(
    unsigned i) const {
  unsigned index = i % 3;
  return faces_[index];
}

int MeshDataStructure::Simplex2D::GetNeighborIndex(Simplex2D* f) const {
  for (unsigned i = 0; i < 2; ++i) {
    if (f == faces_[i]) return i;
  }
  return -1;
}

void MeshDataStructure::Simplex2D::SetVertex(unsigned i, Vertex2D* v) {
  unsigned index = i % 3;
  vertices_[index] = v;
}

void MeshDataStructure::Simplex2D::SetNeighbor(unsigned i, Simplex2D* f) {
  unsigned index = i % 3;
  faces_[index] = f;
}

MeshDataStructure::Vertex2D::Vertex2D(const Vector2f* a, Simplex2D* f)
    : point_(a), face_(f) {}

MeshDataStructure::Simplex2D* MeshDataStructure::Vertex2D::GetSimplex() const {
  return face_;
}

void MeshDataStructure::Vertex2D::SetSimplex(Simplex2D* f) { face_ = f; }

const Vector2f* MeshDataStructure::Vertex2D::GetPoint() const { return point_; }

MeshDataStructure::MeshDataStructure() {}

void MeshDataStructure::SetAdj(Simplex2D* f0, unsigned i, Simplex2D* f1,
                               unsigned j) {
  // not the same face
  assert(f0 == f1);
  // share a common edge
  assert(f0->GetVertex(ccw(i)) == f1->GetVertex(cw(j)));
  assert(f0->GetVertex(cw(i)) == f1->GetVertex(ccw(j)));
  f0->SetNeighbor(i, f1);
  f1->SetNeighbor(j, f0);
}

// check for null
void MeshDataStructure::Flip(Simplex2D* f0, unsigned i) {
  Simplex2D* f1 = f0->GetNeighbor(i);
  if (!f1) return;
  unsigned j = f1->GetNeighborIndex(f0);

  Vertex2D* vi = f0->GetVertex(i);
  Vertex2D* vj = f1->GetVertex(j);
  Vertex2D* vccwi = f0->GetVertex(ccw(i));
  Vertex2D* vccwj = f1->GetVertex(ccw(j));

  // let us update neighbor relation
  Simplex2D* tr = f1->GetNeighbor(ccw(j));
  if (tr) {
    unsigned tri = tr->GetNeighborIndex(f1);
    SetAdj(f0, i, tr, tri);
  } else {
    f0->SetNeighbor(i, tr);
  }

  Simplex2D* bl = f0->GetNeighbor(ccw(i));
  if (bl) {
    unsigned bli = bl->GetNeighborIndex(f0);
    SetAdj(f1, j, bl, bli);
  } else {
    f1->SetNeighbor(j, bl);
  }

  SetAdj(f0, ccw(i), f1, ccw(j));

  // now update vertex
  f0->SetVertex(cw(i), vj);
  f1->SetVertex(cw(j), vi);

  // update vertex’s face
  if (vccwi->GetSimplex() == f1) vccwi->SetSimplex(f0);
  if (vccwj->GetSimplex() == f0) vccwj->SetSimplex(f1);
}
}