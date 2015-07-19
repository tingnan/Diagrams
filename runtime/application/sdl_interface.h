// Copyright 2015 Native Client Authors.

#ifndef RUNTIME_APPLICATION_SDL_INTERFACE_H_
#define RUNTIME_APPLICATION_SDL_INTERFACE_H_


#include <SDL2/SDL.h>
#ifdef __APPLE__
#include <SDL2/SDL_opengl.h>
#else
#include <SDL2/SDL_opengles2.h>
#endif

#include <memory>

#include "physics/world.h"
#include "gl/drawer.h"
#include "gl/sdl_ttf_drawer.h"


namespace diagrammar {

class Application {
 public:  
  Application();
  ~Application();
  bool Init(int, int);
  void Render();
  
 private:
  void HandleEvents();
  bool HandleMessage(const Json::Value&);
  bool LoadFont();
  // For debugging purpose only
  void RenderID();
  bool app_running_ = true;
  SDL_Window* window_;
  SDL_GLContext gl_context_;
  GLProgram gl_program_;
  TTF_Font* font_;
  World world_;
  bool draw_poly_ = false;
  bool draw_path_ = true;
  bool draw_text_ = false;
  std::unique_ptr<Canvas<NodePolyDrawer> > poly_drawers_;
  std::unique_ptr<Canvas<NodePathDrawer> > path_drawers_;
  std::unique_ptr<TextDrawer> text_drawer_;
};

}  // namespace diagrammar

#endif  // RUNTIME_APPLICATION_SDL_INTERFACE_H_
