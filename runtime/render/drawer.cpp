#include "drawer.h"
#include "utility/world_parser.h"
#include <iostream>

namespace {
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
}
namespace diagrammar {

void Drawer::LoadShaders() {
  // read shaders
  GLuint vert_shader_id = CompileShaderFromFile("vert.shader", GL_VERTEX_SHADER);
  GLuint frag_shader_id = CompileShaderFromFile("frag.shader", GL_FRAGMENT_SHADER);
  // now we can link the program

  program_id_ = glCreateProgram();
  glAttachShader(program_id_, vert_shader_id);
  glAttachShader(program_id_, frag_shader_id);
  glLinkProgram(program_id_);

  GLint result;
  glGetProgramiv(program_id_, GL_LINK_STATUS, &result);
  if (result != GL_TRUE) {
    ProgarmErrorHanlder(program_id_);
  }

  glDeleteShader(vert_shader_id);
  glDeleteShader(frag_shader_id);
}


void Drawer::GenPathBuffers() {
  path_vao_.reserve(world_.GetNumNodes());
  path_color_vbo_.reserve(world_.GetNumNodes());
  path_vertex_vbo_.reserve(world_.GetNumNodes());
  path_array_size_.reserve(world_.GetNumNodes());
  for (size_t index = 0; index < world_.GetNumNodes(); ++index) {
    Node* node_ptr = world_.GetNodeByIndex(index);
    for (size_t geo_idx = 0; geo_idx < node_ptr->GetGeometryCount(); ++geo_idx) {
      // flattern
      assert(node_ptr != nullptr);
      ComplexShape2D* geoptr = node_ptr->GetGeometry(geo_idx);
      unsigned num_paths = 1 + geoptr->GetNumHoles();

      for (size_t pa_idx = 0; pa_idx < num_paths; ++pa_idx) {
        path_node_.emplace_back(node_ptr);
        GLuint vao_id;
        glGenVertexArrays(1, &vao_id);
        glBindVertexArray(vao_id);
        path_vao_.emplace_back(vao_id);

        GLuint vert_vbo;
        glGenBuffers(1, &vert_vbo);
        path_vertex_vbo_.emplace_back(vert_vbo);
        glBindBuffer(GL_ARRAY_BUFFER, vert_vbo);
        // serialize the vector of 2S points to GLfloat
        // the rendering is in three dimension
        const std::vector<Vec2f>& points = pa_idx == 0 ? geoptr->GetPath() : geoptr->GetHole(pa_idx - 1);

        std::vector<GLfloat> vertices(points.size() * 3);
        for (size_t j = 0; j < points.size(); ++j) {
          vertices[3 * j + 0] = points[j](0);
          vertices[3 * j + 1] = points[j](1);
          vertices[3 * j + 2] = 0;
        }
        if (geoptr->IsPathClosed()) {
          vertices.emplace_back(points[0](0));
          vertices.emplace_back(points[0](1));
          vertices.emplace_back(0);
        }

        // count for closed loops
        path_array_size_.emplace_back(vertices.size() / 3);

        glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(GLfloat),
                     vertices.data(), GL_STATIC_DRAW);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, nullptr);
        glEnableVertexAttribArray(0);

        GLuint color_vbo;
        glGenBuffers(1, &color_vbo);
        path_color_vbo_.emplace_back(color_vbo);
        glBindBuffer(GL_ARRAY_BUFFER, color_vbo);

        std::vector<GLfloat> colors(vertices.size());
        for (size_t j = 0; j < colors.size() / 3; ++j) {
          colors[3 * j + 0] = 1.f;
          colors[3 * j + 1] = 1.f;
          colors[3 * j + 2] = 1.f;
        }
      
        glBufferData(GL_ARRAY_BUFFER, colors.size() * sizeof(GLfloat),
                     colors.data(), GL_STATIC_DRAW);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, nullptr);
        glEnableVertexAttribArray(1);
      }
      glBindVertexArray(0);
    }
  }
}

void Drawer::GenPolyBuffers() {
  poly_vao_.reserve(world_.GetNumNodes());
  poly_color_vbo_.reserve(world_.GetNumNodes());
  poly_vertex_vbo_.reserve(world_.GetNumNodes());
  poly_array_size_.reserve(world_.GetNumNodes());
  for (size_t index = 0; index < world_.GetNumNodes(); ++index) {
    for (size_t geo_idx = 0; geo_idx < world_.GetNodeByIndex(index)->GetGeometryCount(); ++geo_idx) {
      poly_node_.emplace_back(world_.GetNodeByIndex(index));
      ComplexShape2D* geo_ptr = world_.GetNodeByIndex(index)->GetGeometry(geo_idx);
      // only draw the first geometry
      GLuint vao_id;
      glGenVertexArrays(1, &vao_id);
      glBindVertexArray(vao_id);
      poly_vao_.emplace_back(vao_id);

      GLuint vert_vbo;
      glGenBuffers(1, &vert_vbo);
      glBindBuffer(GL_ARRAY_BUFFER, vert_vbo);
      poly_vertex_vbo_.emplace_back(vert_vbo);
      std::vector<Triangle2D> triangles = geo_ptr->Triangulate();
      std::vector<GLfloat> vertices(triangles.size() * 3 * 3);
      for (size_t j = 0; j < triangles.size(); ++j) {
        vertices[9 * j + 0] = triangles[j].p0(0);
        vertices[9 * j + 1] = triangles[j].p0(1);
        vertices[9 * j + 3] = triangles[j].p1(0);
        vertices[9 * j + 4] = triangles[j].p1(1);
        vertices[9 * j + 6] = triangles[j].p2(0);
        vertices[9 * j + 7] = triangles[j].p2(1);
      }
      glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(GLfloat),
                   vertices.data(), GL_STATIC_DRAW);
      glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, nullptr);
      glEnableVertexAttribArray(0);
      poly_array_size_.emplace_back(triangles.size() * 3);

      GLuint color_vbo;
      glGenBuffers(1, &color_vbo);
      glBindBuffer(GL_ARRAY_BUFFER, color_vbo);
      poly_color_vbo_.emplace_back(color_vbo);
      std::vector<GLfloat> colors(triangles.size() * 9);
      for (size_t j = 0; j < triangles.size(); ++j) {
        colors[9 * j + 0] = 1.f;
        colors[9 * j + 1] = 0.f;
        colors[9 * j + 2] = 0.f;
        colors[9 * j + 3] = 0.f;
        colors[9 * j + 4] = 1.f;
        colors[9 * j + 5] = 0.f;
        colors[9 * j + 6] = 0.f;
        colors[9 * j + 7] = 0.f;
        colors[9 * j + 8] = 1.f;
      }
      glBufferData(GL_ARRAY_BUFFER, colors.size() * sizeof(GLfloat),
                   colors.data(), GL_STATIC_DRAW);
      glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, nullptr);
      glEnableVertexAttribArray(1);
    }
  }
}

void Drawer::UpdateBuffer() {
  // only update when the shape changes
}

void Drawer::DrawPaths() {
  for (size_t i = 0; i < path_vao_.size(); ++i) {
    // get transformation
    Eigen::Isometry3f transform(Eigen::Isometry3f::Identity());
    transform.linear().topLeftCorner<2, 2>() = path_node_[i]->GetRotationMatrix();
    transform.translation().head<2>() = path_node_[i]->GetPosition();
    GLint loc = glGetUniformLocation(program_id_, "modelToEyeMat");
    glUniformMatrix4fv(loc, 1, false, transform.data());

    // draw path
    glBindVertexArray(path_vao_[i]);
    glDrawArrays(GL_LINE_STRIP, 0, path_array_size_[i]);
  }
}

void Drawer::DrawPolygons() {
  for (size_t i = 0; i < poly_vao_.size(); ++i) {
    Eigen::Isometry3f transform(Eigen::Isometry3f::Identity());
    transform.linear().topLeftCorner<2, 2>() = poly_node_[i]->GetRotationMatrix();
    transform.translation().head<2>() = poly_node_[i]->GetPosition();
    GLint loc = glGetUniformLocation(program_id_, "modelToEyeMat");
    glUniformMatrix4fv(loc, 1, false, transform.data());
    glBindVertexArray(poly_vao_[i]);
    glDrawArrays(GL_TRIANGLES, 0, poly_array_size_[i]);
  }
}

void Drawer::Draw() {
  UpdateBuffer();
  glClearColor(0.3f, 0.3f, 0.3f, 0.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glUseProgram(program_id_);
  DrawPaths();
  //_DrawPolygons();
  glBindVertexArray(0);
  glUseProgram(0);
}
}