// Copyright 2015 Native Client Authors.

#ifndef RUNTIME_SDL_SDL_INTERFACE_H_
#define RUNTIME_SDL_SDL_INTERFACE_H_


#include <SDL2/SDL.h>
#include <SDL2/SDL_opengles2.h>

#include <memory>

#include "physics/world.h"
#include "gl/drawer.h"


namespace diagrammar {

class Application {
 public:  
  Application();
  ~Application();
  bool Init(int, int);
  void Render();
  
 private:
  void HandleEvents();
  bool LoadFont();
  bool app_running_ = true;
  SDL_Window* window_;
  SDL_GLContext gl_context_;
  World world_;
  std::unique_ptr<Canvas<NodePolyDrawer> > drawer_;
};

}  // namespace diagrammar

#endif  // RUNTIME_SDL_SDL_INTERFACE_H_
