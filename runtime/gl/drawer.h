// Copyright 2015 Native Client Authors.
#ifndef RUNTIME_SDL_DRAWER_H_
#define RUNTIME_SDL_DRAWER_H_

#include <GLES2/gl2.h>

#include <vector>
#include <unordered_map>

#include "physics/node.h"

namespace diagrammar {

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

class NodeDrawer {
 public:
  virtual void Draw(GLProgram program, GLfloat scale) = 0;
  virtual ~NodeDrawer() = default;
 protected:
  std::vector<GLuint> vertex_size_;
  // Buffer id for vertex and color array
  std::vector<GLuint> vertex_buffer_;
  std::vector<GLuint> vertex_color_buffer_;
  Node* node_ = nullptr;
};

// Draw all the path related to a node
class NodePathDrawer : public NodeDrawer {
 public:
  // Set a node before calling Draw()
  explicit NodePathDrawer(Node*);
  // Use a precompiled program and a scale to draw
  void Draw(GLProgram program, GLfloat scale);

 private:
  void GenPathBuffer(const Path&, bool);
  void GenBuffers();

};


// Draw all the polygons related to a node
class NodePolyDrawer : public NodeDrawer {
 public:
  // Set a node before calling Draw()
  explicit NodePolyDrawer(Node*);
  void Draw(GLProgram program, GLfloat scale);

 private:
  
  void GenTriangleBuffer(const TriangleMesh&);
  void GenBuffers();

  std::vector<GLuint> index_buffer_;
  std::vector<GLuint> index_size_;

};

template<class DrawerType>
class Canvas {
 public:
  Canvas(float scale);
  void AddNode(Node* node);
  void RemoveNodeByID(int id);
  void Draw();
 
 private:

  void LoadProgram();
  
  GLProgram program_;
  float scale_;
  std::unordered_map<int, std::unique_ptr<DrawerType> > drawers_;
};

}  // namespace diagrammar

#endif  // RUNTIME_SDL_NACL_DRAWER_H_
