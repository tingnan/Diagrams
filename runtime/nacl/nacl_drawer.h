#ifndef _DIAGRAMMAR_NACLDRAW3D_
#define _DIAGRAMMAR_NACLDRAW3D_

#include <GLES2/gl2.h>
#include "physics/world.h"

namespace diagrammar {
// currently I am not going to polute other
// files with opengl headers, so all opengl calls and drawings
// are confined here. We can discuss which approach is better

// we have a drawer class that draws opengl primitives
// the drawer class visit the world, examine the objects and the draw all things
class NaClDrawer {
 private:
  const World& world_;
  GLuint program_id_;

  std::vector<GLuint> path_vert_vbo_;
  std::vector<GLuint> path_vert_size_;
  std::vector<GLuint> path_color_vbo_;
  std::vector<Node*> path_node_;

  std::vector<GLuint> poly_vert_vbo_;
  std::vector<GLuint> poly_vert_size_;
  std::vector<GLuint> poly_color_vbo_;
  std::vector<Node*> poly_node_;

 public:
  NaClDrawer(const World& world);
  // not copyable
  NaClDrawer(const NaClDrawer& other) = delete;
  // can move
  NaClDrawer(NaClDrawer&&) = default;
  void LoadShaders();

  // generate vao and vbo for boundary path
  void GenPathBuffers();
  // generate vao and vbo for interior polygons
  void GenPolyBuffers();

  void UpdateBuffer();
  void Draw();
  void DrawPaths();
  void DrawPolygons();
};
}

#endif