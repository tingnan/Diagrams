// Copyright 2015 Native Client Authors.

#include <ft2build.h>
#include FT_FREETYPE_H


#include <iostream>
#include <vector>
#include <string>

#include "sdl/drawer.h"
#include "geometry/geometry2.h"
#include "utility/world_parser.h"
#include "utility/stl_memory.h"

namespace {

const GLuint kVertDim = 4;

const char kFragShaderSource[] =
    "precision mediump float;\n"
    "varying vec4 v_color;\n"
    "varying vec2 tex_coord;\n"
    "uniform sampler2D texture_sampler;\n"
    "void main() {\n"
    "  gl_FragColor = v_color + texture2D(texture_sampler, tex_coord);\n"
    "}\n";

const char kVertShaderSource[] =
    "uniform float scale;\n"
    "uniform mat4 u_mvp;\n"
    "attribute vec4 position;\n"
    "attribute vec4 color;\n"
    "varying vec4 v_color;\n"
    "varying vec2 tex_coord;\n"
    "void main() {\n"
    "  gl_Position = u_mvp * vec4(position.xy, 0.0, 1.0);\n"
    "  gl_Position.xy = gl_Position.xy * scale;\n"
    "  tex_coord = position.zw;\n"
    "  v_color = color;\n"
    "}\n";

void ShaderErrorHandler(GLuint shader_id) {
  GLint log_size;
  glGetShaderiv(shader_id, GL_INFO_LOG_LENGTH, &log_size);
  log_size = log_size > 0 ? log_size : 1024;
  std::vector<char> compileLog(log_size);
  glGetShaderInfoLog(shader_id, log_size, nullptr, compileLog.data());
  std::cerr << "Compile Error: " << compileLog.data() << std::endl;
}

void ProgarmErrorHanlder(GLuint program_id) {
  GLint log_size;
  glGetProgramiv(program_id, GL_INFO_LOG_LENGTH, &log_size);
  log_size = log_size > 0 ? log_size : 1024;
  std::vector<char> errorMessage(log_size);
  glGetProgramInfoLog(program_id, log_size, nullptr, errorMessage.data());
  std::cerr << "Linking Error: " << errorMessage.data() << std::endl;
}

GLuint CompileShaderFromSource(const char* data, GLenum type) {
  GLuint shader_id = glCreateShader(type);
  glShaderSource(shader_id, 1, &data, nullptr);
  glCompileShader(shader_id);
  GLint result = GL_FALSE;
  glGetShaderiv(shader_id, GL_COMPILE_STATUS, &result);
  if (result != GL_TRUE) {
    ShaderErrorHandler(shader_id);
    return 0;
  }
  return shader_id;
}

GLuint CompileShaderFromFile(const char* fname, GLenum type) {
  std::string shader_text = diagrammar::Stringify(fname);
  const char* shader_text_cstr = shader_text.c_str();
  return CompileShaderFromSource(shader_text_cstr, type);
}

void RenderString2D(FT_Face fc, std::string s, float x, float y, float sx,
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

}  // namespace

namespace diagrammar {

NodePathDrawer::NodePathDrawer(Node* node) {
  node_ = node;
  GenBuffers();
}

void NodePathDrawer::GenPolylineBuffer(const Polyline& polyline, bool is_closed) {
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
  glBufferData(GL_ARRAY_BUFFER, colors.size() * sizeof(GLfloat),
               colors.data(), GL_STATIC_DRAW);  
}

void NodePathDrawer::GenBuffers() {
  assert(node_ != nullptr);

  // All closed path
  for (size_t geo_idx = 0; geo_idx < node_->GetNumPolygon();
         ++geo_idx) {
    Polygon* geoptr = node_->GetPolygon(geo_idx);
    unsigned num_paths = 1 + geoptr->holes.size();
    for (size_t pa_idx = 0; pa_idx < num_paths; ++pa_idx) {
      const std::vector<Vector2f>& polyline =
          pa_idx == 0 ? geoptr->path : geoptr->holes[pa_idx - 1];
      GenPolylineBuffer(polyline, true);
    }
  }

  // All open path
  for (size_t geo_idx = 0; geo_idx < node_->GetNumPolyline(); ++geo_idx) {
    GenPolylineBuffer(*(node_->GetPolyline(geo_idx)), false);
  }

}

void NodePathDrawer::Draw(GLProgram program, float scale) {
  assert(node_);
  for (size_t i = 0; i < vertex_buffer_.size(); ++i) {
    glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer_[i]);
    glVertexAttribPointer(program.vertex_loc, kVertDim, GL_FLOAT, GL_FALSE, 0,
                          0);
    glEnableVertexAttribArray(program.vertex_loc);

    glBindBuffer(GL_ARRAY_BUFFER, vertex_color_buffer_[i]);
    glVertexAttribPointer(program.color_loc, kVertDim, GL_FLOAT, GL_FALSE, 0,
                          0);
    glEnableVertexAttribArray(program.color_loc);

    Isometry3f u_mvp(Isometry3f::Identity());
    u_mvp.linear().topLeftCorner<2, 2>() =
        node_->GetRotationMatrix();
    u_mvp.translation().head<2>() = node_->GetPosition();
    glUniformMatrix4fv(program.u_mvp_loc, 1, false, u_mvp.data());

    glUniform1f(program.scale_loc, scale);
    glDrawArrays(GL_LINE_STRIP, 0, vertex_size_[i]);
  }
}


NodePolyDrawer::NodePolyDrawer(Node* node) {
  node_ = node;
  GenBuffers();
}


void NodePolyDrawer::GenBuffers() {
  assert(node_ != nullptr);

  // All closed path
  for (size_t geo_idx = 0; geo_idx < node_->GetNumPolygon();
         ++geo_idx) {
    Polygon* geo_ptr = node_->GetPolygon(geo_idx);
    std::vector<Triangle> triangles = TriangulatePolygon(*geo_ptr);
    GenTriangleBuffer(triangles);
  }

  // All open path
  for (size_t geo_idx = 0; geo_idx < node_->GetNumPolyline(); ++geo_idx) {
    Polyline* geo_ptr = node_->GetPolyline(geo_idx);
    std::vector<Triangle> triangles = TriangulatePolyline(*geo_ptr, 1.5);
    GenTriangleBuffer(triangles);
  }

}

void NodePolyDrawer::GenTriangleBuffer(const std::vector<Triangle>& triangles) {
  
  size_t num_vertices = triangles.size() * 3;
  vertex_size_.emplace_back(num_vertices);

  GLuint v_buffer;
  glGenBuffers(1, &v_buffer);
  vertex_buffer_.emplace_back(v_buffer);
  glBindBuffer(GL_ARRAY_BUFFER, v_buffer);
  
  // the vertes array
  std::vector<GLfloat> vert_array;
  vert_array.reserve(num_vertices * kVertDim);
  for (auto tr : triangles) {
    vert_array.emplace_back(tr.p0(0));
    vert_array.emplace_back(tr.p0(1));
    vert_array.emplace_back(0);
    vert_array.emplace_back(1);
    vert_array.emplace_back(tr.p1(0));
    vert_array.emplace_back(tr.p1(1));
    vert_array.emplace_back(0);
    vert_array.emplace_back(1);
    vert_array.emplace_back(tr.p2(0));
    vert_array.emplace_back(tr.p2(1));
    vert_array.emplace_back(0);
    vert_array.emplace_back(1);
  }
  glBufferData(GL_ARRAY_BUFFER, vert_array.size() * sizeof(GLfloat),
               vert_array.data(), GL_STATIC_DRAW);

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
  glBufferData(GL_ARRAY_BUFFER, colors.size() * sizeof(GLfloat),
               colors.data(), GL_STATIC_DRAW);
}

void NodePolyDrawer::Draw(GLProgram program, float scale) {
  assert(node_);
  for (size_t i = 0; i < vertex_buffer_.size(); ++i) {
    glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer_[i]);
    glVertexAttribPointer(program.vertex_loc, kVertDim, GL_FLOAT, GL_FALSE, 0,
                          0);
    glEnableVertexAttribArray(program.vertex_loc);

    glBindBuffer(GL_ARRAY_BUFFER, vertex_color_buffer_[i]);
    glVertexAttribPointer(program.color_loc, kVertDim, GL_FLOAT, GL_FALSE, 0,
                          0);
    glEnableVertexAttribArray(program.color_loc);

    Isometry3f u_mvp(Isometry3f::Identity());
    u_mvp.linear().topLeftCorner<2, 2>() =
        node_->GetRotationMatrix();
    u_mvp.translation().head<2>() = node_->GetPosition();
    glUniformMatrix4fv(program.u_mvp_loc, 1, false, u_mvp.data());

    glUniform1f(program.scale_loc, scale);

    glDrawArrays(GL_TRIANGLES, 0, vertex_size_[i]);
  }  
}

template<class DrawerType>
Canvas<DrawerType>::Canvas(float scale) : scale_(scale) {
  LoadProgram();
}


template<class DrawerType>
void Canvas<DrawerType>::AddNode(Node* node) {
  drawers_[node->id()] = make_unique<DrawerType>(node);
}

template<class DrawerType>
void Canvas<DrawerType>::RemoveNodeByID(int id) {
  if (drawers_.find(id) != drawers_.end()) {
    drawers_.erase(id);
  }
}

template<class DrawerType>
void Canvas<DrawerType>::Draw() {
  glClearColor(0.3, 0.3, 0.3, 1);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glUseProgram(program_.program_id);
  for (auto itr = drawers_.begin(); itr != drawers_.end(); ++itr) {
    itr->second->Draw(program_, scale_);
  }
}

template<class DrawerType>
void Canvas<DrawerType>::LoadProgram() {
  GLuint vert_shader_id =
      CompileShaderFromSource(kVertShaderSource, GL_VERTEX_SHADER);
  GLuint frag_shader_id =
      CompileShaderFromSource(kFragShaderSource, GL_FRAGMENT_SHADER);
  // now we can link the program

  GLuint program_id = glCreateProgram();
  glAttachShader(program_id, vert_shader_id);
  glAttachShader(program_id, frag_shader_id);
  glLinkProgram(program_id);
  GLint result;
  glGetProgramiv(program_id, GL_LINK_STATUS, &result);
  if (result != GL_TRUE) {
    ProgarmErrorHanlder(program_id);
  }
  glDeleteShader(vert_shader_id);
  glDeleteShader(frag_shader_id);

  program_.tex_loc = glGetUniformLocation(program_id, "texture_sampler");
  program_.u_mvp_loc = glGetUniformLocation(program_id, "u_mvp");
  program_.scale_loc = glGetUniformLocation(program_id, "scale");
  program_.color_loc = glGetAttribLocation(program_id, "color");
  program_.vertex_loc = glGetAttribLocation(program_id, "position");
  program_.program_id = program_id;  
}

template class Canvas<NodePathDrawer>;
template class Canvas<NodePolyDrawer>;


/*
void NaClDrawer::LoadShaders() {
  // compile vert shader
  GLuint vert_shader_id =
      CompileShaderFromSource(kVertShaderSource, GL_VERTEX_SHADER);
  GLuint frag_shader_id =
      CompileShaderFromSource(kFragShaderSource, GL_FRAGMENT_SHADER);
  // now we can link the program

  GLuint program_id = glCreateProgram();
  glAttachShader(program_id, vert_shader_id);
  glAttachShader(program_id, frag_shader_id);
  glLinkProgram(program_id);
  GLint result;
  glGetProgramiv(program_id, GL_LINK_STATUS, &result);
  if (result != GL_TRUE) {
    ProgarmErrorHanlder(program_id);
  }
  glDeleteShader(vert_shader_id);
  glDeleteShader(frag_shader_id);

  program_.tex_loc = glGetUniformLocation(program_id, "texture_sampler");
  program_.u_mvp_loc = glGetUniformLocation(program_id, "u_mvp");
  program_.scale_loc = glGetUniformLocation(program_id, "scale");
  program_.color_loc = glGetAttribLocation(program_id, "color");
  program_.vertex_loc = glGetAttribLocation(program_id, "position");
  program_.program_id = program_id;
}

void NaClDrawer::GenTextBuffers() {
  FT_Library ft;
  if (FT_Init_FreeType(&ft)) {
    std::cerr << "Could not init freetype library\n";
    return;
  }
  if (FT_New_Face(ft, "DejaVuSans.ttf", 0, &freetype_face_)) {
    std::cerr << "Could not open font\n";
    return;
  }
  FT_Set_Pixel_Sizes(freetype_face_, 0, 32);

  glGenBuffers(1, &text_vbo_);
  glGenTextures(1, &text_tex_);
  glBindTexture(GL_TEXTURE_2D, text_tex_);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
  glBindTexture(GL_TEXTURE_2D, 0);
}

void NaClDrawer::DrawTexts() {
  GLProgram program = program_;
  glBindBuffer(GL_ARRAY_BUFFER, text_vbo_);
  glVertexAttribPointer(program.vertex_loc, 4, GL_FLOAT, GL_FALSE, 0, 0);
  glEnableVertexAttribArray(program.vertex_loc);

  Isometry3f u_mvp(Isometry3f::Identity());
  glUniformMatrix4fv(program.u_mvp_loc, 1, false, u_mvp.data());

  // set texture
  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D, text_tex_);
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
  glUniform1i(program.tex_loc, 0);

  std::string s("hello world!");
  RenderString2D(freetype_face_, s, 0, 0, 1, 1);
}

void NaClDrawer::Draw() {
  glClearColor(0.3, 0.3, 0.3, 1);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glUseProgram(program_.program_id);
  DrawPolygons();
  DrawPaths();
  // DrawTexts();
}
*/
}  // namespace diagrammar
