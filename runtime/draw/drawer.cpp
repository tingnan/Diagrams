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

/*
void RenderString(FT_Face fc, std::string s, float x, float y, float sx,
                    float sy) {
  for (auto ch : s) {
    if (FT_Load_Char(fc, ch, FT_LOAD_RENDER)) continue;
    FT_GlyphSlot glyph = fc->glyph;
    glTexImage2D(GL_TEXTURE_2D, 0, GL_ALPHA, glyph->bitmap.width,
                 glyph->bitmap.rows, 0, GL_ALPHA, GL_UNSIGNED_BYTE,
                 glyph->bitmap.buffer);
    // now we have added a texture
    // let us create a box, the x and y are cursor position
    // sx and sy are scale in each direction
    GLfloat xpos = x + glyph->bitmap_left * sx;
    GLfloat ypos = y + glyph->bitmap_top * sy;
    GLfloat w = glyph->bitmap.width * sx;
    GLfloat h = glyph->bitmap.rows * sy;
    GLfloat char_box[16] = {xpos, ypos,     0, 0, xpos + w, ypos,     1, 0,
                            xpos, ypos - h, 0, 1, xpos + w, ypos - h, 1, 1};
    glBufferData(GL_ARRAY_BUFFER, sizeof(char_box), char_box, GL_DYNAMIC_DRAW);
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    x += (glyph->advance.x >> 6) * sx;
    y += (glyph->advance.y >> 6) * sy;
  }
}
*/
}  // namespace

namespace diagrammar {

NodePathDrawer::NodePathDrawer(const Node* node) {
  node_ = node;
  GenBuffers();
}

NodePathDrawer::~NodePathDrawer() {
  glDeleteBuffers(vertex_buffer_.size(), vertex_buffer_.data());
  glDeleteBuffers(vertex_color_buffer_.size(), vertex_color_buffer_.data());
}

void NodePathDrawer::GenPathBuffer(const Path2D& polyline, bool is_closed) {
  GLuint vert_vbo;
  glGenBuffers(1, &vert_vbo);
  vertex_buffer_.emplace_back(vert_vbo);
  glBindBuffer(GL_ARRAY_BUFFER, vert_vbo);
  size_t num_vertices = polyline.size();

  std::vector<GLfloat> vertices(num_vertices * kVertDim);
  for (size_t j = 0; j < polyline.size(); ++j) {
    vertices[kVertDim * j + 0] = polyline[j](0);
    vertices[kVertDim * j + 1] = polyline[j](1);
    vertices[kVertDim * j + 2] = 0;
    vertices[kVertDim * j + 3] = 1;
  }
  if (is_closed) {
    vertices.emplace_back(polyline[0](0));
    vertices.emplace_back(polyline[0](1));
    vertices.emplace_back(0);
    vertices.emplace_back(0);
    num_vertices++;
  }
  glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(GLfloat),
               vertices.data(), GL_STATIC_DRAW);

  // count for closed loops
  vertex_size_.emplace_back(num_vertices);

  GLuint color_vbo;
  glGenBuffers(1, &color_vbo);
  vertex_color_buffer_.emplace_back(color_vbo);
  glBindBuffer(GL_ARRAY_BUFFER, color_vbo);

  std::vector<GLfloat> colors(vertices.size());
  for (size_t j = 0; j < num_vertices; ++j) {
    colors[kVertDim * j + 0] = 1;
    colors[kVertDim * j + 1] = 1;
    colors[kVertDim * j + 2] = 1;
    colors[kVertDim * j + 3] = 1;
  }
  glBufferData(GL_ARRAY_BUFFER, colors.size() * sizeof(GLfloat), colors.data(),
               GL_STATIC_DRAW);
}

void NodePathDrawer::GenBuffers() {
  assert(node_ != nullptr);

  for (auto& shape_ptr : node_->collision_shapes) {
    switch (shape_ptr->shape_type) {
      case Shape2DType::kDisk: {
        auto sphere_ptr = dynamic_cast<Disk2D*>(shape_ptr.get());
        const size_t num_vertices = 30;
        Path2D polyline;
        for (size_t i = 0; i < num_vertices; ++i) {
          float theta = float(i) / num_vertices * M_PI * 2;
          polyline.emplace_back(cos(theta) * sphere_ptr->radius,
                                sin(theta) * sphere_ptr->radius);
        }
        GenPathBuffer(polyline, true);
      } break;
      case Shape2DType::kPolygon: {
        auto poly_ptr = dynamic_cast<Polygon2D*>(shape_ptr.get());
        unsigned num_paths = 1 + poly_ptr->holes.size();
        for (size_t i = 0; i < num_paths; ++i) {
          const std::vector<Vector2f>& polyline =
              i == 0 ? poly_ptr->path : poly_ptr->holes[i - 1];
          GenPathBuffer(polyline, true);
        }
      } break;
      case Shape2DType::kPolyLine: {
        auto line_ptr = dynamic_cast<Line2D*>(shape_ptr.get());
        GenPathBuffer(line_ptr->path, false);
      } break;
      default:
        break;
    }
  }
}

void NodePathDrawer::Draw(GLProgram program, Camera* camera) {
  assert(node_);
  for (size_t i = 0; i < vertex_buffer_.size(); ++i) {
    glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer_[i]);
    glVertexAttribPointer(program.vertex, kVertDim, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(program.vertex);

    glBindBuffer(GL_ARRAY_BUFFER, vertex_color_buffer_[i]);
    glVertexAttribPointer(program.color, kVertDim, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(program.color);

    Matrix4f u_mvp(Matrix4f::Identity());
    u_mvp.topLeftCorner<3, 3>() = node_->frame.GetRotationMatrix();
    u_mvp.col(3).head<3>() = node_->frame.GetTranslation();
    u_mvp = camera->GetViewProjection() * u_mvp;
    glUniformMatrix4fv(program.u_mvp, 1, false, u_mvp.data());

    glDrawArrays(GL_LINE_STRIP, 0, vertex_size_[i]);
  }
  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glDisableVertexAttribArray(program.vertex);
  glDisableVertexAttribArray(program.color);
}

NodePolyDrawer::NodePolyDrawer(const Node* node) {
  node_ = node;
  GenBuffers();
}

NodePolyDrawer::~NodePolyDrawer() {
  glDeleteBuffers(vertex_buffer_.size(), vertex_buffer_.data());
  glDeleteBuffers(vertex_color_buffer_.size(), vertex_color_buffer_.data());
  glDeleteBuffers(index_buffer_.size(), index_buffer_.data());
}

void NodePolyDrawer::GenBuffers() {
  assert(node_ != nullptr);
  for (auto& shape_ptr : node_->collision_shapes) {
    switch (shape_ptr->shape_type) {
      case Shape2DType::kDisk: {
        auto sphere_ptr = dynamic_cast<Disk2D*>(shape_ptr.get());
        const size_t num_vertices = 30;
        Polygon2D polygon;
        for (size_t i = 0; i < num_vertices; ++i) {
          float theta = float(i) / num_vertices * M_PI * 2;
          polygon.path.emplace_back(cos(theta) * sphere_ptr->radius,
                                    sin(theta) * sphere_ptr->radius);
        }
        auto mesh = TriangulatePolygon(polygon);
        GenTriangleBuffer(mesh);
      } break;
      case Shape2DType::kPolygon: {
        auto poly_ptr = dynamic_cast<Polygon2D*>(shape_ptr.get());
        auto mesh = TriangulatePolygon(*poly_ptr);
        GenTriangleBuffer(mesh);
      } break;
      case Shape2DType::kPolyLine: {
        auto line_ptr = dynamic_cast<Line2D*>(shape_ptr.get());
        auto mesh = TriangulatePolyline(line_ptr->path, 1.5);
        GenTriangleBuffer(mesh);
      } break;
      default:
        break;
    }
  }
}

// TODO(tingnan) fix rendering
void NodePolyDrawer::GenTriangleBuffer(const TriangleMesh2D& mesh) {
  size_t num_vertices = mesh.vertices.size();
  vertex_size_.emplace_back(num_vertices);

  GLuint v_buffer;
  glGenBuffers(1, &v_buffer);
  vertex_buffer_.emplace_back(v_buffer);
  glBindBuffer(GL_ARRAY_BUFFER, v_buffer);

  // the vertes array
  std::vector<GLfloat> vert_array;
  vert_array.reserve(num_vertices * kVertDim);
  for (auto& vt : mesh.vertices) {
    vert_array.emplace_back(vt(0));
    vert_array.emplace_back(vt(1));
    vert_array.emplace_back(0);
    vert_array.emplace_back(0);
  }
  glBufferData(GL_ARRAY_BUFFER, vert_array.size() * sizeof(GLfloat),
               vert_array.data(), GL_STATIC_DRAW);

  GLuint i_buffer;
  glGenBuffers(1, &i_buffer);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, i_buffer);
  index_buffer_.emplace_back(i_buffer);
  std::vector<GLuint> indices;
  indices.reserve(mesh.faces.size() * 3);
  for (auto& face : mesh.faces) {
    for (size_t vt_idx = 0; vt_idx < 3; ++vt_idx) {
      indices.emplace_back(face[vt_idx]);
    }
  }
  index_size_.emplace_back(indices.size());
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(GLuint),
               indices.data(), GL_STATIC_DRAW);

  GLuint c_buffer;
  glGenBuffers(1, &c_buffer);
  vertex_color_buffer_.emplace_back(c_buffer);
  glBindBuffer(GL_ARRAY_BUFFER, c_buffer);
  std::vector<GLfloat> colors(num_vertices * kVertDim);
  // each triangle is labeled with r g b
  for (size_t cid = 0; cid < num_vertices; ++cid) {
    if (cid % 3 == 0) {
      colors[kVertDim * cid + 0] = 1;
      colors[kVertDim * cid + 1] = 1;
      colors[kVertDim * cid + 2] = 0;
      colors[kVertDim * cid + 3] = 1;
    }
    if (cid % 3 == 1) {
      colors[kVertDim * cid + 0] = 1;
      colors[kVertDim * cid + 1] = 0;
      colors[kVertDim * cid + 2] = 1;
      colors[kVertDim * cid + 3] = 1;
    }
    if (cid % 3 == 2) {
      colors[kVertDim * cid + 0] = 0;
      colors[kVertDim * cid + 1] = 1;
      colors[kVertDim * cid + 2] = 1;
      colors[kVertDim * cid + 3] = 1;
    }
  }
  glBufferData(GL_ARRAY_BUFFER, colors.size() * sizeof(GLfloat), colors.data(),
               GL_STATIC_DRAW);
}

void NodePolyDrawer::Draw(GLProgram program, Camera* camera) {
  assert(node_);
  for (size_t i = 0; i < vertex_buffer_.size(); ++i) {
    glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer_[i]);
    glVertexAttribPointer(program.vertex, kVertDim, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(program.vertex);

    glBindBuffer(GL_ARRAY_BUFFER, vertex_color_buffer_[i]);
    glVertexAttribPointer(program.color, kVertDim, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(program.color);

    Matrix4f u_mvp(Matrix4f::Identity());
    u_mvp.topLeftCorner<3, 3>() = node_->frame.GetRotationMatrix();
    u_mvp.col(3).head<3>() = node_->frame.GetTranslation();
    u_mvp = camera->GetViewProjection() * u_mvp;
    glUniformMatrix4fv(program.u_mvp, 1, false, u_mvp.data());

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, index_buffer_[i]);
    glDrawElements(GL_TRIANGLES, index_size_[i], GL_UNSIGNED_INT, 0);
  }
  glDisableVertexAttribArray(program.vertex);
  glDisableVertexAttribArray(program.color);
}

// move back to header
template <class DrawerType>
NodeGroupDrawer<DrawerType>::NodeGroupDrawer(GLProgram program, Camera* camera)
    : program_(program), camera_(camera) {}

template <class DrawerType>
void NodeGroupDrawer<DrawerType>::AddNode(const Node* node) {
  drawers_[node->id] = make_unique<DrawerType>(node);
}

template <class DrawerType>
void NodeGroupDrawer<DrawerType>::RemoveNodeByID(int id) {
  if (drawers_.find(id) != drawers_.end()) {
    drawers_.erase(id);
  }
}

template <class DrawerType>
void NodeGroupDrawer<DrawerType>::Draw() {
  glUseProgram(program_.pid);
  for (auto itr = drawers_.begin(); itr != drawers_.end(); ++itr) {
    itr->second->Draw(program_, camera_);
  }
  glUseProgram(0);
}

// to prevent linking error
template class NodeGroupDrawer<NodePathDrawer>;
template class NodeGroupDrawer<NodePolyDrawer>;

CanvasDrawer::CanvasDrawer(Camera* camera) : camera_(camera) {
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
  GLfloat resolution[2] = {600, 600};
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
