#ifndef SDL_NACL_DRAWER_
#define SDL_NACL_DRAWER_
#include "physics/world.h"
#include <ft2build.h>
#include FT_FREETYPE_H
#include <GLES2/gl2.h>

namespace diagrammar {
// currently I am not going to polute other
// files with opengl headers, so all opengl calls and drawings
// are confined here. We can discuss which approach is better

// we have a drawer class that draws opengl primitives
// the drawer class visit the world, examine the objects and the draw all things

struct GLProgram {
  // texture
  GLuint tex_loc;
  // scaling
  GLuint scale_loc;
  // transformation matrix
  GLuint u_mvp_loc;
  // color location
  GLuint color_loc;
  // vertex location
  GLuint vertex_loc;
  // program
  GLuint program_id;
};

class NaClDrawer {
 private:
  const World& world_;
  std::vector<GLuint> path_vert_vbo_;
  std::vector<GLuint> path_vert_size_;
  std::vector<GLuint> path_color_vbo_;
  std::vector<Node*> path_node_;

  std::vector<GLuint> poly_vert_vbo_;
  std::vector<GLuint> poly_vert_size_;
  std::vector<GLuint> poly_color_vbo_;
  std::vector<Node*> poly_node_;
  FT_Face freetype_face_;

  GLuint text_tex_;
  GLuint text_vbo_;

  GLProgram program_;
 private:
  void LoadShaders();
  // generate vbo for boundary path
  void GenPathBuffers();
  // generate vbo for interior polygons
  void GenPolyBuffers();
  // generate vbo for fonts
  void GenTextBuffers();
  // drawing
  void DrawPaths();
  void DrawPolygons();
  void DrawTexts();
 public:
  NaClDrawer(const World& world);
  // not copyable
  NaClDrawer(const NaClDrawer& other) = delete;
  // can move
  NaClDrawer(NaClDrawer&&) = default;
  void Draw();
};
}

#endif