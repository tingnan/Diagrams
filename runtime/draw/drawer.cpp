// Copyright 2015 Native Client Authors.

#include <iostream>
#include <vector>
#include <string>

#include "draw/drawer.h"
#include "draw/camera.h"
#include "geometry/geometry2.h"
#include "utility/world_parser.h"
#include "utility/stl_memory.h"

namespace {
const GLuint kVertDim = 4;
const std::array<GLfloat, 16> kQuad = {
    {-1, 1, 0.0, 0.0, 1, 1, 0.0, 0.0, 1, -1, 0.0, 0.0, -1, -1, 0.0, 0.0}};
const std::array<GLuint, 6> kQuadIndices = {{0, 1, 2, 2, 3, 0}};

void GLEnableVertexAttrib(GLuint attrib_loc, GLenum buffer_target,
                          GLuint buffer_id, GLuint dimension = 4,
                          GLenum data_type = GL_FLOAT) {
  glBindBuffer(buffer_target, buffer_id);
  glVertexAttribPointer(attrib_loc, dimension, data_type, GL_FALSE, 0, 0);
  glEnableVertexAttribArray(attrib_loc);
}

diagrammar::Matrix4f GLGetNodeViewProjection(diagrammar::Camera *camera,
                                             const diagrammar::Node *node) {
  diagrammar::Matrix4f u_mvp(diagrammar::Matrix4f::Identity());
  u_mvp.topLeftCorner<3, 3>() = node->frame.GetRotationMatrix();
  u_mvp.col(3).head<3>() = node->frame.GetTranslation();
  u_mvp = camera->GetViewProjection() * u_mvp;
  return u_mvp;
}

}  // namespace

namespace diagrammar {

NodePathDrawer::NodePathDrawer(const Node *node) {
  node_ = node;
  GenBuffers();
}

NodePathDrawer::~NodePathDrawer() {
  glDeleteBuffers(vertex_buffer_.size(), vertex_buffer_.data());
  glDeleteBuffers(vertex_color_buffer_.size(), vertex_color_buffer_.data());
}

void NodePathDrawer::GenPathBuffer(const Path2D &polyline, bool is_closed) {
  GLuint vert_vbo;
  glGenBuffers(1, &vert_vbo);
  glBindBuffer(GL_ARRAY_BUFFER, vert_vbo);
  size_t num_vertices = polyline.size();
  if (is_closed) num_vertices++;
  std::vector<GLfloat> vertices = SerializePath2D(polyline, is_closed);
  glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(GLfloat),
               vertices.data(), GL_STATIC_DRAW);

  GLuint color_vbo;
  glGenBuffers(1, &color_vbo);
  glBindBuffer(GL_ARRAY_BUFFER, color_vbo);
  std::vector<GLfloat> colors(vertices.size(), 1);
  glBufferData(GL_ARRAY_BUFFER, colors.size() * sizeof(GLfloat), colors.data(),
               GL_STATIC_DRAW);

  // Store the buffer id and buffer size
  vertex_buffer_.emplace_back(vert_vbo);
  vertex_size_.emplace_back(num_vertices);
  vertex_color_buffer_.emplace_back(color_vbo);
}

void NodePathDrawer::GenBuffers() {
  assert(node_ != nullptr);
  for (auto &shape_ptr : node_->collision_shapes) {
    switch (shape_ptr->shape_type) {
      case Shape2DType::kDisk: {
        auto disk_ptr = dynamic_cast<Disk2D *>(shape_ptr.get());
        const size_t num_vertices = 30;
        Path2D polyline = SketchDiskEdge(*disk_ptr, num_vertices);
        GenPathBuffer(polyline, true);
      } break;
      case Shape2DType::kPolygon: {
        auto poly_ptr = dynamic_cast<Polygon2D *>(shape_ptr.get());
        GenPathBuffer(poly_ptr->path, true);
        for (auto &hole : poly_ptr->holes) {
          GenPathBuffer(hole, true);
        }
      } break;
      case Shape2DType::kPolyLine: {
        auto line_ptr = dynamic_cast<Line2D *>(shape_ptr.get());
        GenPathBuffer(line_ptr->path, false);
      } break;
      default:
        break;
    }
  }
}

void NodePathDrawer::Draw(GLProgram program, Camera *camera,
                          Vector2f resolution, float current_time) {
  assert(node_);
  for (size_t i = 0; i < vertex_buffer_.size(); ++i) {
    GLEnableVertexAttrib(program.vertex, GL_ARRAY_BUFFER, vertex_buffer_[i]);
    GLEnableVertexAttrib(program.color, GL_ARRAY_BUFFER,
                         vertex_color_buffer_[i]);

    Matrix4f u_mvp = GLGetNodeViewProjection(camera, node_);
    glUniformMatrix4fv(program.u_mvp, 1, false, u_mvp.data());

    glDrawArrays(GL_LINE_STRIP, 0, vertex_size_[i]);
  }
  glDisableVertexAttribArray(program.vertex);
  glDisableVertexAttribArray(program.color);
}

NodePolyDrawer::NodePolyDrawer(const Node *node) {
  node_ = node;
  GenBuffers();
}

NodePolyDrawer::~NodePolyDrawer() {
  glDeleteBuffers(vertex_buffer_.size(), vertex_buffer_.data());
  glDeleteBuffers(vertex_color_buffer_.size(), vertex_color_buffer_.data());
  glDeleteBuffers(tri_index_buffer_.size(), tri_index_buffer_.data());
}

void NodePolyDrawer::GenBuffers() {
  assert(node_ != nullptr);
  for (auto &shape_ptr : node_->collision_shapes) {
    GenTriangleBuffer(shape_ptr.get());
  }
}

void NodePolyDrawer::GenTriangleBuffer(const CollisionShape2D *shape) {
  GLTriangleMesh gl_mesh = GLTriangulate2DShape2D(shape);

  GLuint v_buffer;
  glGenBuffers(1, &v_buffer);

  glBindBuffer(GL_ARRAY_BUFFER, v_buffer);
  glBufferData(GL_ARRAY_BUFFER, gl_mesh.vertices.size() * sizeof(GLfloat),
               gl_mesh.vertices.data(), GL_STATIC_DRAW);

  GLuint i_buffer;
  glGenBuffers(1, &i_buffer);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, i_buffer);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, gl_mesh.indices.size() * sizeof(GLuint),
               gl_mesh.indices.data(), GL_STATIC_DRAW);

  GLuint c_buffer;
  glGenBuffers(1, &c_buffer);
  glBindBuffer(GL_ARRAY_BUFFER, c_buffer);
  glBufferData(GL_ARRAY_BUFFER, gl_mesh.colors.size() * sizeof(GLfloat),
               gl_mesh.colors.data(), GL_STATIC_DRAW);

  tri_index_size_.emplace_back(gl_mesh.indices.size());
  tri_index_buffer_.emplace_back(i_buffer);
  vertex_size_.emplace_back(gl_mesh.vertices.size());
  vertex_buffer_.emplace_back(v_buffer);
  vertex_color_buffer_.emplace_back(c_buffer);
}

void NodePolyDrawer::Draw(GLProgram program, Camera *camera,
                          Vector2f resolution, float current_time) {
  assert(node_);
  for (size_t i = 0; i < vertex_buffer_.size(); ++i) {
    GLEnableVertexAttrib(program.vertex, GL_ARRAY_BUFFER, vertex_buffer_[i]);
    GLEnableVertexAttrib(program.color, GL_ARRAY_BUFFER,
                         vertex_color_buffer_[i]);

    Matrix4f u_mvp = GLGetNodeViewProjection(camera, node_);
    glUniformMatrix4fv(program.u_mvp, 1, false, u_mvp.data());

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, tri_index_buffer_[i]);
    glDrawElements(GL_TRIANGLES, tri_index_size_[i], GL_UNSIGNED_INT, 0);
  }
  glDisableVertexAttribArray(program.vertex);
  glDisableVertexAttribArray(program.color);
}

NodeBuldgedDrawer::NodeBuldgedDrawer(const Node *node) {
  node_ = node;
  GenBuffers();
}

NodeBuldgedDrawer::~NodeBuldgedDrawer() {
  glDeleteBuffers(vertex_buffer_.size(), vertex_buffer_.data());
  glDeleteBuffers(vertex_color_buffer_.size(), vertex_color_buffer_.data());
  glDeleteBuffers(tri_index_buffer_.size(), tri_index_buffer_.data());
}

void NodeBuldgedDrawer::GenBuffers() {
  assert(node_ != nullptr);
  for (auto &shape_ptr : node_->collision_shapes) {
    GenTriangleBuffer(shape_ptr.get());
  }
}

void NodeBuldgedDrawer::GenTriangleBuffer(const CollisionShape2D *shape) {
  GLTriangleMesh gl_mesh = GLTriangulate3DShape2D(shape, 10);

  GLuint v_buffer;
  glGenBuffers(1, &v_buffer);

  glBindBuffer(GL_ARRAY_BUFFER, v_buffer);
  glBufferData(GL_ARRAY_BUFFER, gl_mesh.vertices.size() * sizeof(GLfloat),
               gl_mesh.vertices.data(), GL_STATIC_DRAW);

  GLuint i_buffer;
  glGenBuffers(1, &i_buffer);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, i_buffer);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, gl_mesh.indices.size() * sizeof(GLuint),
               gl_mesh.indices.data(), GL_STATIC_DRAW);

  GLuint c_buffer;
  glGenBuffers(1, &c_buffer);
  glBindBuffer(GL_ARRAY_BUFFER, c_buffer);
  glBufferData(GL_ARRAY_BUFFER, gl_mesh.colors.size() * sizeof(GLfloat),
               gl_mesh.colors.data(), GL_STATIC_DRAW);

  GLuint n_buffer;
  glGenBuffers(1, &n_buffer);
  glBindBuffer(GL_ARRAY_BUFFER, n_buffer);
  glBufferData(GL_ARRAY_BUFFER, gl_mesh.normals.size() * sizeof(GLfloat),
               gl_mesh.normals.data(), GL_STATIC_DRAW);

  tri_index_size_.emplace_back(gl_mesh.indices.size());
  tri_index_buffer_.emplace_back(i_buffer);
  vertex_size_.emplace_back(gl_mesh.vertices.size());
  vertex_buffer_.emplace_back(v_buffer);
  vertex_color_buffer_.emplace_back(c_buffer);
  normal_buffer_.emplace_back(n_buffer);
}

void NodeBuldgedDrawer::Draw(GLProgram program, Camera *camera,
                             Vector2f resolution, float current_time) {
  assert(node_);
  glUniform2f(program.resolution, resolution(0), resolution(1));
  GLuint time_pos = glGetUniformLocation(program.pid, "time");
  glUniform1f(time_pos, current_time);
  for (size_t i = 0; i < vertex_buffer_.size(); ++i) {
    GLEnableVertexAttrib(program.vertex, GL_ARRAY_BUFFER, vertex_buffer_[i]);
    GLEnableVertexAttrib(program.normal, GL_ARRAY_BUFFER, normal_buffer_[i]);
    GLEnableVertexAttrib(program.color, GL_ARRAY_BUFFER,
                         vertex_color_buffer_[i]);
    Matrix4f u_mvp = GLGetNodeViewProjection(camera, node_);
    glUniformMatrix4fv(program.u_mvp, 1, false, u_mvp.data());

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, tri_index_buffer_[i]);
    glDrawElements(GL_TRIANGLES, tri_index_size_[i], GL_UNSIGNED_INT, 0);
  }
  glDisableVertexAttribArray(program.vertex);
  glDisableVertexAttribArray(program.color);
  glDisableVertexAttribArray(program.normal);
}

SphereDrawer::SphereDrawer(const Node *node) {
  node_ = node;
  GenBuffers();
}
SphereDrawer::~SphereDrawer() {
  glDeleteBuffers(vertex_buffer_.size(), vertex_buffer_.data());
  glDeleteBuffers(vertex_color_buffer_.size(), vertex_color_buffer_.data());
}

void SphereDrawer::GenBuffers() {
  vertex_size_.emplace_back(0);
  tri_index_size_.emplace_back(0);

  std::vector<GLfloat> vert_array;
  std::vector<GLfloat> colors;
  std::vector<GLuint> indices;
  for (auto &shape_ptr : node_->collision_shapes) {
    if (shape_ptr->shape_type == Shape2DType::kDisk) {
      auto sphere_ptr = dynamic_cast<Disk2D *>(shape_ptr.get());
      float sphere_radius = sphere_ptr->radius;
      std::vector<GLfloat> quad_vertices(16, 0);
      std::vector<GLfloat> color(16, 1);

      for (size_t i = 0; i < 16; i += 4) {
        quad_vertices[i + 0] = kQuad[i + 0] * sphere_radius;
        quad_vertices[i + 1] = kQuad[i + 1] * sphere_radius;
        quad_vertices[i + 2] = 0;
        quad_vertices[i + 3] = sphere_radius;
      }

      vert_array.insert(vert_array.end(), quad_vertices.begin(),
                        quad_vertices.end());

      colors.insert(colors.end(), color.begin(), color.end());
      std::vector<GLuint> quad_indices(kQuadIndices.begin(),
                                       kQuadIndices.end());
      for (auto &index : quad_indices) {
        index += tri_index_size_[0];
      }
      indices.insert(indices.end(), quad_indices.begin(), quad_indices.end());

      // Update num_vertices
      vertex_size_[0] += 4;
      tri_index_size_[0] += 6;
    }
  }

  vertex_buffer_.resize(1);
  glGenBuffers(1, &vertex_buffer_[0]);
  vertex_color_buffer_.resize(1);
  glGenBuffers(1, &vertex_color_buffer_[0]);
  tri_index_buffer_.resize(1);
  glGenBuffers(1, &tri_index_buffer_[0]);

  // now bind buffers and upload data
  glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer_[0]);
  glBufferData(GL_ARRAY_BUFFER, vert_array.size() * sizeof(GLfloat),
               vert_array.data(), GL_STATIC_DRAW);
  glBindBuffer(GL_ARRAY_BUFFER, vertex_color_buffer_[0]);
  glBufferData(GL_ARRAY_BUFFER, colors.size() * sizeof(GLfloat), colors.data(),
               GL_STATIC_DRAW);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, tri_index_buffer_[0]);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(GLuint),
               indices.data(), GL_STATIC_DRAW);
}
void SphereDrawer::Draw(GLProgram program, Camera *camera, Vector2f resolution,
                        float current_time) {
  assert(node_);
  assert(vertex_buffer_.size() == 1);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  for (size_t i = 0; i < vertex_buffer_.size(); ++i) {
    glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer_[i]);
    glVertexAttribPointer(program.vertex, kVertDim, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(program.vertex);

    glBindBuffer(GL_ARRAY_BUFFER, vertex_color_buffer_[i]);
    glVertexAttribPointer(program.color, kVertDim, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(program.color);

    Matrix4f u_mvp(Matrix4f::Identity());
    u_mvp.col(3).head<3>() = node_->frame.GetTranslation();
    u_mvp = camera->GetView() * u_mvp;
    u_mvp.topLeftCorner<3, 3>() = Matrix3f::Identity();
    u_mvp = camera->GetProjection() * u_mvp;
    // Now we just align the frame with camera frame
    glUniformMatrix4fv(program.u_mvp, 1, false, u_mvp.data());

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, tri_index_buffer_[i]);
    glDrawElements(GL_TRIANGLES, tri_index_size_[i], GL_UNSIGNED_INT, 0);
  }
  glDisableVertexAttribArray(program.vertex);
  glDisableVertexAttribArray(program.color);
}

// move back to header
template <class DrawerType>
NodeGroupDrawer<DrawerType>::NodeGroupDrawer(GLProgram program, Camera *camera,
                                             Vector2f resolution)
    : program_(program), camera_(camera), view_port_(resolution) {}

template <class DrawerType>
void NodeGroupDrawer<DrawerType>::AddNode(const Node *node) {
  drawers_[node->id] = make_unique<DrawerType>(node);
}

template <class DrawerType>
void NodeGroupDrawer<DrawerType>::RemoveNodeByID(int id) {
  if (drawers_.find(id) != drawers_.end()) {
    drawers_.erase(id);
  }
}

template <class DrawerType>
void NodeGroupDrawer<DrawerType>::Draw(float current_time) {
  glUseProgram(program_.pid);
  for (auto itr = drawers_.begin(); itr != drawers_.end(); ++itr) {
    itr->second->Draw(program_, camera_, view_port_, current_time);
  }
  glUseProgram(0);
}

// to prevent linking error
template class NodeGroupDrawer<NodePathDrawer>;
template class NodeGroupDrawer<NodePolyDrawer>;
template class NodeGroupDrawer<SphereDrawer>;
template class NodeGroupDrawer<NodeBuldgedDrawer>;

CanvasDrawer::CanvasDrawer(Camera *camera, Vector2f resolution)
    : camera_(camera), view_port_(resolution) {
  GenBuffers();
  std::string vert_shader = Stringify("bgshader.vert");
  std::string frag_shader = Stringify("bgshader.frag");
  program_.pid = CreateGLProgram(vert_shader.c_str(), frag_shader.c_str());
  program_.u_mvp = glGetUniformLocation(program_.pid, "u_mvp");
  program_.color = glGetAttribLocation(program_.pid, "color");
  program_.vertex = glGetAttribLocation(program_.pid, "vertex");
  time_ = glGetUniformLocation(program_.pid, "time");
  resolution_ = glGetUniformLocation(program_.pid, "resolution");
}

CanvasDrawer::~CanvasDrawer() {
  glDeleteBuffers(1, &vert_buffer_);
  glDeleteBuffers(1, &vert_indice_);
}

void CanvasDrawer::GenBuffers() {
  glGenBuffers(1, &vert_buffer_);
  glBindBuffer(GL_ARRAY_BUFFER, vert_buffer_);
  std::array<GLfloat, 16> vert_array = kQuad;
  for (size_t i = 0; i < 16; i += 4) {
    vert_array[i + 0] = vert_array[i + 0] * 10000;
    vert_array[i + 1] = vert_array[i + 1] * 10000;
    vert_array[i + 2] = -1000;
  }

  glBufferData(GL_ARRAY_BUFFER, vert_array.size() * sizeof(GLfloat),
               vert_array.data(), GL_STATIC_DRAW);

  glGenBuffers(1, &vert_indice_);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vert_indice_);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(GLuint) * kQuadIndices.size(),
               kQuadIndices.data(), GL_STATIC_DRAW);
  glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void CanvasDrawer::Draw(float curr_time) {
  glUseProgram(program_.pid);

  // Resolution of the ocean and the time fluctuation
  GLfloat resolution[2] = {view_port_(0), view_port_(1)};
  glUniform2fv(resolution_, 1, resolution);
  glUniform1f(time_, curr_time);

  glBindBuffer(GL_ARRAY_BUFFER, vert_buffer_);
  glVertexAttribPointer(program_.vertex, kVertDim, GL_FLOAT, GL_FALSE, 0, 0);
  glEnableVertexAttribArray(program_.vertex);

  Matrix4f u_mvp(Matrix4f::Identity());
  u_mvp = camera_->GetViewProjection() * u_mvp;
  glUniformMatrix4fv(program_.u_mvp, 1, false, u_mvp.data());

  // scale
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vert_indice_);
  glDrawElements(GL_TRIANGLES, kQuadIndices.size(), GL_UNSIGNED_INT, 0);

  glUseProgram(0);
}

}  // namespace diagrammar
