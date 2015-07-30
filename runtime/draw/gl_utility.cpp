// Copyright 2015 Native Client Authors.

#include <vector>
#include <string>
#include <iostream>

#include "utility/world_parser.h"
#include "draw/gl_utility.h"

namespace {
const char kFragShaderSource[] =
    "precision mediump float;\n"
    "varying vec4 v_color;\n"
    "void main() {\n"
    "  gl_FragColor = v_color;\n"
    "}\n";

const char kVertShaderSource[] =
    "uniform mat4 u_mvp;\n"
    "attribute vec4 vertex;\n"
    "attribute vec4 normal;\n"
    "attribute vec4 color;\n"
    "varying vec4 v_color;\n"
    "void main() {\n"
    "  gl_Position = u_mvp * vec4(vertex.xyz, 1.0);\n"
    "  v_color = vec4(color.xyz, 1.0);\n"
    "}\n";

}  // namespace

namespace diagrammar {

void ShaderErrorHandler(GLuint shader_id) {
  GLint log_size;
  glGetShaderiv(shader_id, GL_INFO_LOG_LENGTH, &log_size);
  log_size = log_size > 0 ? log_size : 1024;
  std::vector<char> compileLog(log_size);
  glGetShaderInfoLog(shader_id, log_size, nullptr, compileLog.data());
  std::cerr << "Compile Error: " << shader_id << " " << compileLog.data()
            << std::endl;
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

GLuint CreateGLProgram(const char* vert_shader_src,
                       const char* frag_shader_src) {
  GLuint vert_shader_id =
      CompileShaderFromSource(vert_shader_src, GL_VERTEX_SHADER);
  GLuint frag_shader_id =
      CompileShaderFromSource(frag_shader_src, GL_FRAGMENT_SHADER);
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

  return program_id;
}

GLProgram LoadDefaultGLProgram() {
  GLProgram program;
  program.pid = CreateGLProgram(kVertShaderSource, kFragShaderSource);
  program.u_mvp = glGetUniformLocation(program.pid, "u_mvp");
  program.color = glGetAttribLocation(program.pid, "color");
  program.normal = glGetAttribLocation(program.pid, "normal");
  program.vertex = glGetAttribLocation(program.pid, "vertex");
  return program;
}

}  // namespace diagrammar
