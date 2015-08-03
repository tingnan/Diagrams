// Copyright 2015 Native Client Authors.
#ifndef RUNTIME_DRAW_DRAWER_H_
#define RUNTIME_DRAW_DRAWER_H_

#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GLES2/gl2.h>
#endif

#include <memory>
#include <vector>
#include <string>
#include <unordered_map>

#include "physics/node.h"
#include "draw/gl_utility.h"

namespace diagrammar {

class Camera;

class NodeDrawer {
 public:
  virtual void Draw(GLProgram program, Camera *camera, Vector2f resolution,
                    float current_time) = 0;
  virtual ~NodeDrawer() = default;

 protected:
  std::vector<GLuint> vertex_size_;
  std::vector<GLuint> vertex_buffer_;
  std::vector<GLuint> vertex_color_buffer_;
  const Node *node_ = nullptr;
};

// Draw all the path related to a node
class NodePathDrawer : public NodeDrawer {
 public:
  // Set a node before calling Draw()
  explicit NodePathDrawer(const Node *node);
  ~NodePathDrawer();
  // Use a precompiled program and a scale to draw
  void Draw(GLProgram program, Camera *camera, Vector2f resolution,
            float current_time);

 private:
  void GenPathBuffer(const Path2D &, bool);
  void GenBuffers();
};

// Draw all the polygons related to a node
class NodePolyDrawer : public NodeDrawer {
 public:
  // Set a node before calling Draw()
  explicit NodePolyDrawer(const Node *node);
  ~NodePolyDrawer();
  void Draw(GLProgram program, Camera *camera, Vector2f resolution,
            float current_time);

 private:
  void GenTriangleBuffer(const CollisionShape2D *shape);
  void GenBuffers();

  std::vector<GLuint> tri_index_buffer_;
  std::vector<GLuint> tri_index_size_;
};

class NodeBuldgedDrawer : public NodeDrawer {
 public:
  // Set a node before calling Draw()
  explicit NodeBuldgedDrawer(const Node *node);
  ~NodeBuldgedDrawer();
  void Draw(GLProgram program, Camera *camera, Vector2f resolution,
            float current_time);

 private:
  void GenTriangleBuffer(const CollisionShape2D *shape);
  void GenBuffers();

  std::vector<GLuint> normal_buffer_;
  std::vector<GLuint> tri_index_buffer_;
  std::vector<GLuint> tri_index_size_;
};

class SphereDrawer : public NodeDrawer {
 public:
  explicit SphereDrawer(const Node *node);
  ~SphereDrawer();
  void Draw(GLProgram program, Camera *camera, Vector2f resolution,
            float current_time);

 private:
  void GenBuffers();
  std::vector<GLuint> tri_index_buffer_;
  std::vector<GLuint> tri_index_size_;
};

template <class DrawerType>
class NodeGroupDrawer {
 public:
  NodeGroupDrawer(GLProgram program, Camera *camera, Vector2f resolution);
  void AddNode(const Node *node);
  void RemoveNodeByID(id_t id);
  void Draw(float current_time);

  void ChangeGLProgram(GLProgram program) { program_ = program; }
  void ChangeCamera(Camera *camera) { camera_ = camera; }
  void ChangeResolution(Vector2f resolution) { view_port_ = resolution; }

 private:
  GLProgram program_;
  Camera *camera_ = nullptr;
  Vector2f view_port_;
  std::unordered_map<id_t, std::unique_ptr<DrawerType>> drawers_;
};

class CanvasDrawer {
 public:
  CanvasDrawer(Camera *camera, Vector2f resolution);
  ~CanvasDrawer();
  void Draw(float current_time);
  void ChangeResolution(Vector2f resolution) { view_port_ = resolution; }
  void ChangeCamera(Camera *camera) { camera_ = camera; }

 private:
  void GenBuffers();
  GLProgram program_;
  GLuint resolution_;
  GLuint time_;
  GLuint vert_buffer_;
  GLuint vert_indice_;
  Camera *camera_ = nullptr;
  Vector2f view_port_;
};

}  // namespace diagrammar

#endif  // RUNTIME_DRAW_DRAWER_H_
