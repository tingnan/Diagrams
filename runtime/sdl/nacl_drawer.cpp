#include "nacl_drawer.h"
#include "utility/world_parser.h"
#include <iostream>

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
}

namespace diagrammar {

NaClDrawer::NaClDrawer(const World& world) : world_(world) {
  LoadShaders();
  GenTextBuffers();
  GenPathBuffers();
  GenPolyBuffers();
}

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

void NaClDrawer::GenPathBuffers() {
  for (size_t index = 0; index < world_.GetNumNodes(); ++index) {
    Node* node_ptr = world_.GetNodeByIndex(index);
    // loop through geometries
    for (size_t geo_idx = 0; geo_idx < node_ptr->GetGeometryCount();
         ++geo_idx) {
      assert(node_ptr != nullptr);
      ComplexShape2D* geoptr = node_ptr->GetGeometry(geo_idx);
      unsigned num_paths = 1 + geoptr->GetNumHoles();
      // loop through the paths and holes
      for (size_t pa_idx = 0; pa_idx < num_paths; ++pa_idx) {
        GLuint vert_vbo;
        glGenBuffers(1, &vert_vbo);
        path_vert_vbo_.emplace_back(vert_vbo);
        glBindBuffer(GL_ARRAY_BUFFER, vert_vbo);
        // serialize the vector of 2S points to GLfloat
        // the rendering is in three dimension
        const std::vector<Vector2f>& points =
            pa_idx == 0 ? geoptr->GetPath() : geoptr->GetHole(pa_idx - 1);
        size_t num_vertices = points.size();
        std::vector<GLfloat> vertices(num_vertices * kVertDim);
        for (size_t j = 0; j < points.size(); ++j) {
          vertices[kVertDim * j + 0] = points[j](0);
          vertices[kVertDim * j + 1] = points[j](1);
          vertices[kVertDim * j + 2] = 0;
          vertices[kVertDim * j + 3] = 1;
        }
        if (geoptr->IsPathClosed()) {
          vertices.emplace_back(points[0](0));
          vertices.emplace_back(points[0](1));
          vertices.emplace_back(0);
          vertices.emplace_back(0);
          num_vertices++;
        }
        glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(GLfloat),
                     vertices.data(), GL_STATIC_DRAW);
        // count for closed loops
        path_vert_size_.emplace_back(num_vertices);

        GLuint color_vbo;
        glGenBuffers(1, &color_vbo);
        path_color_vbo_.emplace_back(color_vbo);
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
      poly_color_vbo_.emplace_back(c_buffer);
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

      // now push back the node info
      poly_node_.emplace_back(node_ptr);
    }
  }
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

void NaClDrawer::DrawPaths() {
  GLProgram program = program_;
  const float kGolbalScale = 1.f / 600.f;
  for (size_t i = 0; i < path_vert_vbo_.size(); ++i) {
    glBindBuffer(GL_ARRAY_BUFFER, path_vert_vbo_[i]);
    glVertexAttribPointer(program.vertex_loc, kVertDim, GL_FLOAT, GL_FALSE, 0,
                          0);
    glEnableVertexAttribArray(program.vertex_loc);

    glBindBuffer(GL_ARRAY_BUFFER, path_color_vbo_[i]);
    glVertexAttribPointer(program.color_loc, kVertDim, GL_FLOAT, GL_FALSE, 0,
                          0);
    glEnableVertexAttribArray(program.color_loc);

    Isometry3f transform(Isometry3f::Identity());
    transform.linear().topLeftCorner<2, 2>() =
        path_node_[i]->GetRotationMatrix();
    transform.translation().head<2>() = path_node_[i]->GetPosition();
    glUniformMatrix4fv(program.u_mvp_loc, 1, false, transform.data());

    glUniform1f(program.scale_loc, kGolbalScale);

    glDrawArrays(GL_LINE_STRIP, 0, path_vert_size_[i]);
  }
}

void NaClDrawer::DrawPolygons() {
  GLProgram program = program_;
  const float kGolbalScale = 1.f / 600.f;
  for (size_t i = 0; i < poly_vert_vbo_.size(); ++i) {
    glBindBuffer(GL_ARRAY_BUFFER, poly_vert_vbo_[i]);
    glVertexAttribPointer(program.vertex_loc, kVertDim, GL_FLOAT, GL_FALSE, 0,
                          0);
    glEnableVertexAttribArray(program.vertex_loc);

    glBindBuffer(GL_ARRAY_BUFFER, poly_color_vbo_[i]);
    glVertexAttribPointer(program.color_loc, kVertDim, GL_FLOAT, GL_FALSE, 0,
                          0);
    glEnableVertexAttribArray(program.color_loc);

    Isometry3f transform(Isometry3f::Identity());
    transform.linear().topLeftCorner<2, 2>() =
        poly_node_[i]->GetRotationMatrix();
    transform.translation().head<2>() = poly_node_[i]->GetPosition();
    glUniformMatrix4fv(program.u_mvp_loc, 1, false, transform.data());

    glUniform1f(program.scale_loc, kGolbalScale);

    glDrawArrays(GL_TRIANGLES, 0, poly_vert_size_[i]);
  }
}

void NaClDrawer::DrawTexts() {
  GLProgram program = program_;
  glBindBuffer(GL_ARRAY_BUFFER, text_vbo_);
  glVertexAttribPointer(program.vertex_loc, 4, GL_FLOAT, GL_FALSE, 0, 0);
  glEnableVertexAttribArray(program.vertex_loc);

  Isometry3f transform(Isometry3f::Identity());
  glUniformMatrix4fv(program.u_mvp_loc, 1, false, transform.data());

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
}
