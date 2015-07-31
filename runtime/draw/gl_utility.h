// Copyright 2015 Native Client Authors.

#ifndef RUNTIME_DRAW_GL_UTILITY_H
#define RUNTIME_DRAW_GL_UTILITY_H

#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GLES2/gl2.h>
#include <GLES2/gl2ext.h>
#endif

namespace diagrammar {
struct GLProgram {
  GLuint pid;
  // the user provided model_view_projection matrix
  GLuint u_mvp;
  GLuint color;
  GLuint vertex;
  GLuint normal;
  GLuint texture;
  GLuint resolution;
};

GLuint CreateGLProgram(const char* vert_shader_src,
                       const char* frag_shader_src);
GLProgram LoadDefaultGLProgram();

}  // namespace diagrammar

#endif  // RUNTIME_DRAW_GL_UTILITY_H
