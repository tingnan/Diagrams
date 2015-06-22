#include <GLES2/gl2.h>
#include "nacl_drawer.h"
#include "utility/json_parser.h"
#include <iostream>

namespace {

const GLuint kVertDim = 3;

const char kFragShaderSource[] =
    "precision mediump float;\n"
    "varying vec3 v_color;\n"
    "void main() {\n"
    "  gl_FragColor = vec4(v_color, 1);\n"
    "}\n";

// hard coded scaling in the shader
const char kVertexShaderSource[] =
    "uniform mat4 u_mvp;\n"
    "attribute vec3 a_position;\n"
    "attribute vec3 a_color;\n"
    "varying vec3 v_color;\n"
    "void main() {\n"
    "  gl_Position = u_mvp * vec4(a_position, 1.0) * 0.002;\n"
    "  gl_Position.w = 1.0;\n"
    "  v_color = a_color;\n"
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
}

namespace diagrammar {

NaClDrawer::NaClDrawer(const World& world) : world_(world) {
  LoadShaders();
  GenPathBuffers();
  GenPolyBuffers();
}
void NaClDrawer::LoadShaders() {
  // compile vert shader
  GLuint vert_shader_id =
      CompileShaderFromSource(kVertexShaderSource, GL_VERTEX_SHADER);
  GLuint frag_shader_id =
      CompileShaderFromSource(kFragShaderSource, GL_FRAGMENT_SHADER);
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

void NaClDrawer::GenPathBuffers() {
  for (size_t index = 0; index < world_.GetNumNodes(); ++index) {
    Node* node_ptr = world_.GetNodeByIndex(index);
    // loop through geometries
    for (size_t geo_idx = 0; geo_idx < node_ptr->GetGeometryCount();
         ++geo_idx) {
      assert(node_ptr != nullptr);
      Geometry2D* geoptr = node_ptr->GetGeometry(geo_idx);
      unsigned num_paths = 1 + geoptr->GetNumHoles();
      // loop through the paths and holes
      for (size_t pa_idx = 0; pa_idx < num_paths; ++pa_idx) {
        GLuint vert_vbo;
        glGenBuffers(1, &vert_vbo);
        path_vert_vbo_.emplace_back(vert_vbo);
        glBindBuffer(GL_ARRAY_BUFFER, vert_vbo);
        // serialize the vector of 2S points to GLfloat
        // the rendering is in three dimension
        const std::vector<Vec2f>& points =
            pa_idx == 0 ? geoptr->GetPath() : geoptr->GetHole(pa_idx - 1);

        std::vector<GLfloat> vertices(points.size() * kVertDim);
        for (size_t j = 0; j < points.size(); ++j) {
          vertices[kVertDim * j + 0] = points[j](0);
          vertices[kVertDim * j + 1] = points[j](1);
          vertices[kVertDim * j + 2] = 0;
        }
        if (geoptr->IsPathClosed()) {
          vertices.emplace_back(points[0](0));
          vertices.emplace_back(points[0](1));
          vertices.emplace_back(0);
        }
        glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(GLfloat),
                     vertices.data(), GL_STATIC_DRAW);
        // count for closed loops
        path_vert_size_.emplace_back(vertices.size() / kVertDim);

        GLuint color_vbo;
        glGenBuffers(1, &color_vbo);
        path_color_vbo_.emplace_back(color_vbo);
        glBindBuffer(GL_ARRAY_BUFFER, color_vbo);

        std::vector<GLfloat> colors(vertices.size());
        for (size_t j = 0; j < colors.size() / 3; ++j) {
          colors[kVertDim * j + 0] = 1;
          colors[kVertDim * j + 1] = 1;
          colors[kVertDim * j + 2] = 1;
        }
        glBufferData(GL_ARRAY_BUFFER, colors.size() * sizeof(GLfloat),
                     colors.data(), GL_STATIC_DRAW);
        path_node_.emplace_back(node_ptr);
      }
    }
  }
}

void NaClDrawer::GenPolyBuffers() {
  for (size_t index = 0; index < world_.GetNumNodes(); ++index) {
    Node* node_ptr = world_.GetNodeByIndex(index);
    for (size_t geo_idx = 0; geo_idx < node_ptr->GetGeometryCount();
         ++geo_idx) {
      std::vector<Triangle2D> triangles =
          node_ptr->GetGeometry(geo_idx)->Triangulate();
      size_t num_vertices = triangles.size() * 3;

      GLuint v_buffer;
      glGenBuffers(1, &v_buffer);
      poly_vert_vbo_.emplace_back(v_buffer);
      glBindBuffer(GL_ARRAY_BUFFER, v_buffer);
      poly_vert_size_.emplace_back(num_vertices);
      // the vertes array
      std::vector<GLfloat> vert_array;
      vert_array.reserve(num_vertices * kVertDim);
      for (auto tr : triangles) {
        vert_array.emplace_back(tr.p0(0));
        vert_array.emplace_back(tr.p0(1));
        vert_array.emplace_back(0);
        vert_array.emplace_back(tr.p1(0));
        vert_array.emplace_back(tr.p1(1));
        vert_array.emplace_back(0);
        vert_array.emplace_back(tr.p2(0));
        vert_array.emplace_back(tr.p2(1));
        vert_array.emplace_back(0);
      }
      glBufferData(GL_ARRAY_BUFFER, vert_array.size() * sizeof(GLfloat),
                   vert_array.data(), GL_STATIC_DRAW);

      GLuint c_buffer;
      glGenBuffers(1, &c_buffer);
      poly_color_vbo_.emplace_back(c_buffer);
      glBindBuffer(GL_ARRAY_BUFFER, c_buffer);
      std::vector<GLfloat> colors(num_vertices * kVertDim);
      // each triangle is labeled with r g b
      for (size_t cid = 0; cid < num_vertices / 3; ++cid) {
        colors[9 * cid + 0] = 1;
        colors[9 * cid + 1] = 0;
        colors[9 * cid + 2] = 0;
        colors[9 * cid + 3] = 0;
        colors[9 * cid + 4] = 1;
        colors[9 * cid + 5] = 0;
        colors[9 * cid + 6] = 0;
        colors[9 * cid + 7] = 0;
        colors[9 * cid + 8] = 1;
      }
      glBufferData(GL_ARRAY_BUFFER, colors.size() * sizeof(GLfloat),
                   colors.data(), GL_STATIC_DRAW);

      // now push back the node info
      poly_node_.emplace_back(node_ptr);
    }
  }
}

void NaClDrawer::UpdateBuffer() {}

void NaClDrawer::DrawPaths() {
  GLuint position_loc = glGetAttribLocation(program_id_, "a_position");
  GLuint color_loc = glGetAttribLocation(program_id_, "a_color");
  GLuint transform_loc = glGetUniformLocation(program_id_, "u_mvp");
  for (size_t i = 0; i < path_vert_vbo_.size(); ++i) {
    glBindBuffer(GL_ARRAY_BUFFER, path_vert_vbo_[i]);
    glVertexAttribPointer(position_loc, kVertDim, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(position_loc);

    glBindBuffer(GL_ARRAY_BUFFER, path_color_vbo_[i]);
    glVertexAttribPointer(color_loc, kVertDim, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(color_loc);

    Eigen::Isometry3f transform(Eigen::Isometry3f::Identity());
    transform.linear().topLeftCorner<2, 2>() =
        path_node_[i]->GetRotationMatrix();
    transform.translation().head<2>() = path_node_[i]->GetPosition();
    glUniformMatrix4fv(transform_loc, 1, false, transform.data());
    glDrawArrays(GL_LINE_STRIP, 0, path_vert_size_[i]);
  }
}

void NaClDrawer::DrawPolygons() {
  GLuint position_loc = glGetAttribLocation(program_id_, "a_position");
  GLuint color_loc = glGetAttribLocation(program_id_, "a_color");
  GLuint transform_loc = glGetUniformLocation(program_id_, "u_mvp");
  for (size_t i = 0; i < poly_vert_vbo_.size(); ++i) {
    glBindBuffer(GL_ARRAY_BUFFER, poly_vert_vbo_[i]);
    glVertexAttribPointer(position_loc, kVertDim, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(position_loc);

    glBindBuffer(GL_ARRAY_BUFFER, poly_color_vbo_[i]);
    glVertexAttribPointer(color_loc, kVertDim, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(color_loc);

    Eigen::Isometry3f transform(Eigen::Isometry3f::Identity());
    transform.linear().topLeftCorner<2, 2>() =
        poly_node_[i]->GetRotationMatrix();
    transform.translation().head<2>() = poly_node_[i]->GetPosition();
    glUniformMatrix4fv(transform_loc, 1, false, transform.data());

    glDrawArrays(GL_TRIANGLES, 0, poly_vert_size_[i]);
  }
}

void NaClDrawer::Draw() {
  glClearColor(0.3, 0.3, 0.3, 1);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glUseProgram(program_id_);
  DrawPaths();
  // DrawPolygons();
}
}
