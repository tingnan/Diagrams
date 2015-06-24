#ifndef GEOMETRY_TRIANGULATIONDATASTRUCTURE_
#define GEOMETRY_TRIANGULATIONDATASTRUCTURE_
#include "typedefs.h"
#include <vector>
#include <memory>

namespace diagrammar {
// the representation of triangulation
// fix me, it is going to be used later when we are trying to refine
// the delaunay triangulation
class MeshDataStructure {
 public:
  class Vertex2D;

  class Simplex2D {
    // in ccw order
    // constness means we cannot modify the points
    Vertex2D* vertices_[3];
    // it does not have ownership
    Simplex2D* faces_[3];

   public:
    Simplex2D(Vertex2D* a, Vertex2D* b, Vertex2D* c, Simplex2D* fa = nullptr,
              Simplex2D* fb = nullptr, Simplex2D* fc = nullptr);

    // get vertex by index
    Vertex2D* GetVertex(unsigned i) const;
    // returns -1 if the vertex does not belong to the face
    int GetVertexIndex(Vertex2D*) const;

    // Neighbor access functions
    Simplex2D* GetNeighbor(unsigned i) const;
    // return -1 if the Simplex is not a neighbor
    int GetNeighborIndex(Simplex2D*) const;

    // Setting

    void SetVertex(unsigned i, Vertex2D*);
    void SetNeighbor(unsigned i, Simplex2D*);
  };

  class Vertex2D {
    const Vec2f* point_;
    Simplex2D* face_;

   public:
    Vertex2D(const Vec2f* a, Simplex2D* f = nullptr);
    Vertex2D(Vertex2D&&) = default;
    Simplex2D* GetSimplex() const;
    void SetSimplex(Simplex2D*);
    const Vec2f* GetPoint() const;
  };

 private:
  std::vector<std::unique_ptr<Simplex2D> > faces_;
  std::vector<std::unique_ptr<Vertex2D> > vertices_;
  std::vector<std::unique_ptr<Vec2f> > points_;
  friend class Triangulation;

  unsigned static ccw(unsigned i) { return (i + 1) % 3; }

  unsigned static cw(unsigned i) { return (i + 2) % 3; }

  void SetAdj(Simplex2D* f0, unsigned i, Simplex2D* f1, unsigned j);

 public:
  MeshDataStructure();
  void Flip(Simplex2D* face, unsigned index);
};
};

#endif