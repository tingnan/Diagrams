// Copyright 2015 Native Client Authors

#include <array>
#include <iostream>

#include "gl/sdl_ttf_drawer.h"

namespace {
SDL_Surface* StringToSDLSurface(const char* message, TTF_Font* font) {
  SDL_Color color = {255, 255, 255};
  SDL_Surface* ttf_surface = TTF_RenderText_Blended(font, message, color);
  if (ttf_surface == nullptr) {
    std::cerr << "TTF_RenderText: " << SDL_GetError() << std::endl;
    return nullptr;
  }
  // SDL_SaveBMP(ttf_surface, "image.bmp");
  // exit(0);
  return ttf_surface;
}

const std::array<GLfloat, 16> kQuad = 
  {-0.5, 0.5, 0.0, 0.0, 
    0.5, 0.5, 1.0, 0.0,
    0.5,-0.5, 1.0, 1.0,
   -0.5,-0.5, 0.0, 1.0};

const GLuint kQuadIndices[] = {0, 1, 2, 1, 2, 3};

}  // namespace

namespace diagrammar {

TextDrawer::TextDrawer(TTF_Font* font) : font_(font) {
  GenBuffers();
}

void TextDrawer::GenBuffers() {
  glGenTextures(1, &text_buffer_);
  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D, text_buffer_);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  // glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  // glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glBindTexture(GL_TEXTURE_2D, 0);
  
  glGenBuffers(1, &vert_buffer_);
  glGenBuffers(1, &vert_indice_);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vert_indice_);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(kQuadIndices), kQuadIndices,
               GL_STATIC_DRAW);
}

void TextDrawer::Draw(const std::string& text, GLProgram program,
                      Vector2f window_size, float scale) {

  SDL_Surface* surface = StringToSDLSurface(text.c_str(), font_);
  if (surface == nullptr) {
    return;
  }
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glUseProgram(program.program_id);

  
  // texture
  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D, text_buffer_);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, surface->w, surface->h, 0, GL_RGBA,
               GL_UNSIGNED_BYTE, surface->pixels);
  // glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
  glUniform1i(program.texture_loc, 0);

  // vertex
  glBindBuffer(GL_ARRAY_BUFFER, vert_buffer_);
  std::array<GLfloat, 16> vert_array = kQuad;
  for (size_t i = 0; i < 16; i += 4) {
    vert_array[i + 0] = vert_array[i + 0] * float(surface->w);
    vert_array[i + 1] = vert_array[i + 1] * float(surface->h);
  }
  glBufferData(GL_ARRAY_BUFFER, vert_array.size() * sizeof(GLfloat),
               vert_array.data(), GL_STATIC_DRAW);

  glVertexAttribPointer(program.vertex_loc, 4, GL_FLOAT, GL_FALSE, 0, 0);
  glEnableVertexAttribArray(program.vertex_loc);

  // text location
  Isometry3f u_mvp(Isometry3f::Identity());
  glUniformMatrix4fv(program.u_mvp_loc, 1, false, u_mvp.data());

  // scale
  glUniform1f(program.scale_loc, scale);

  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vert_indice_);
  glDrawElements(GL_TRIANGLES, 2, GL_UNSIGNED_INT, 0);
  glDrawArrays(GL_LINE_LOOP, 0, 4);

  glBindTexture(GL_TEXTURE_2D, 0);
  glActiveTexture(0);
  SDL_FreeSurface(surface);
  glUseProgram(0);
}

}  // namespace diagrammar