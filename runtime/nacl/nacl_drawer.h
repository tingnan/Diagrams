// Copyright 2015 Native Client authors
#ifndef RUNTIME_NACL_NACL_DRAWER_
#define RUNTIME_NACL_NACL_DRAWER_

#include "physics/world.h"
#include <GLES2/gl2.h>

namespace diagrammar {
// currently I am not going to polute other
// files with opengl headers, so all opengl calls and drawings
// are confined here. We can discuss which approach is better

// we have a drawer class that draws opengl primitives
// the drawer class visit the world, examine the objects and the draw all things
class NaClDrawer {
 public:
  NaClDrawer(const World& world);
  // not copyable
  NaClDrawer(const NaClDrawer& other) = delete;
  // can move
  NaClDrawer(NaClDrawer&&) = default;
  void Draw();

 private:
  const World& world_;
  void LoadShaders();
  // generate vao and vbo for boundary path
  void GenPathBuffers();
  // generate vao and vbo for interior polygons
  void GenPolyBuffers();
  // drawing
  void DrawPaths();
  void DrawPolygons();

  GLuint program_id_;
  std::vector<GLuint> path_vert_vbo_;
  std::vector<GLuint> path_vert_size_;
  std::vector<GLuint> path_color_vbo_;
  std::vector<Node*> path_node_;

  std::vector<GLuint> poly_vert_vbo_;
  std::vector<GLuint> poly_vert_size_;
  std::vector<GLuint> poly_color_vbo_;
  std::vector<Node*> poly_node_;
};
}  // namespace diagrammar

#endif  // RUNTIME_NACL_NACL_DRAWER_