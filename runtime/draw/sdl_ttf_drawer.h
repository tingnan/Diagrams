// Copyright 2015 Native Client Authors

#ifndef RUNTIME_DRAW_SDL_TTF_DRAWER_H_
#define RUNTIME_DRAW_SDL_TTF_DRAWER_H_

#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#ifdef __APPLE__
#include <SDL2/SDL_opengl.h>
#else
#include <SDL2/SDL_opengles2.h>
#endif

#include <string>

#include "include/matrix_types.h"
#include "draw/gl_utility.h"

namespace diagrammar {

class TextDrawer {
 public:
  explicit TextDrawer(TTF_Font* font);
  // w and h are window sizes
  void Draw(const std::string& text, const Vector2f& pos, class Camera* camera);

 private:
  void GenBuffers();
  TTF_Font* font_;
  GLuint text_buffer_;
  GLuint vert_buffer_;
  GLuint vert_indice_;
  GLuint vert_color_;
  GLProgram program_;
};

}  // namespace diagrammar

#endif  // RUNTIME_DRAW_SDL_TTF_DRAWER_H_
