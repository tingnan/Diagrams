// Copyright 2015 Native Client Authors

#ifndef RUNTIME_GL_SDL_TTF_DRAWER_H_
#define RUNTIME_GL_SDL_TTF_DRAWER_H_

#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#ifdef __APPLE__
#include <SDL2/SDL_opengl.h>
#else
#include <SDL2/SDL_opengles2.h>
#endif

#include <string>

#include "draw/drawer.h"

namespace diagrammar {

class TextDrawer {
 public:
  explicit TextDrawer(TTF_Font* font);
  // w and h are window sizes
  void Draw(const std::string& text, GLProgram program, Vector2f pos, Vector2f window_size,
            float scale);

 private:
  void GenBuffers();
  GLuint text_buffer_;
  GLuint vert_buffer_;
  GLuint vert_indice_;
  GLuint vert_color_;
  TTF_Font* font_;
};

}  // namespace diagrammar

#endif  // RUNTIME_GL_SDL_TTF_DRAWER_H_