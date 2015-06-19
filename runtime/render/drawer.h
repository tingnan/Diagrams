#ifndef _DIAGRAMMAR_DRAW_
#define _DIAGRAMMAR_DRAW_

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <map>
#include "physics/world.h"
namespace diagrammar {
// currently I am not going to polute other
// files with opengl headers, so all opengl calls and drawings
// are confined here. We can discuss which approach is better

// we have a drawer class that draws opengl primitives
// the drawer class visit the world, examine the objects and the draw all things
class Drawer {
  // all opengl transformations all going to be computed using
  // the Eigen libraries
  // we want the drawer only accessible from the window class
  friend class Window;

 private:
  const World& world_;
  std::vector<GLuint> path_vao_;
  std::vector<GLuint> path_vertex_vbo_;
  std::vector<GLuint> path_color_vbo_;
  std::vector<GLuint> path_array_size_;
  std::vector<Node*> path_node_;

  std::vector<GLuint> poly_vao_;
  std::vector<GLuint> poly_vertex_vbo_;
  std::vector<GLuint> poly_color_vbo_;
  std::vector<GLuint> poly_array_size_;
  std::vector<Node*> poly_node_;

  GLuint program_id_;

 private:
  Drawer(const World& world) : world_(world) {}
  // not copyable
  Drawer(const Drawer& other) = delete;
  // can move
  Drawer(Drawer&&) = default;
  void _LoadShaders();

  // generate vao and vbo for boundary path
  void _GenPathBuffers();
  // generate vao and vbo for interior polygons
  void _GenPolyBuffers();

  // not implemented
  void _UpdateBuffer();
  void _Draw();
  void _DrawPaths();
  void _DrawPolygons();

 private:
  // helper function
};

// have a few visitor class that convert the path, geometry to
// opengl primitives (vertices and triangles) so the drawer
// can be called
}

#endif