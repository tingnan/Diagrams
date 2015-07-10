// Copyright 2015 Native Client Authors.

#ifndef RUNTIME_SDL_SDL_INTERFACE_H_
#define RUNTIME_SDL_SDL_INTERFACE_H_

#include <memory>

#include "physics/world.h"
#include "gl/drawer.h"

class SDL_Window;
namespace diagrammar {
class SDLInterfaceOpenGL {
 public:
  
  SDLInterfaceOpenGL();
  ~SDLInterfaceOpenGL();
  bool Init(int, int);
  void Render();
  
 private:
  
  void HandleEvents();
  bool LoadFont();
  bool app_running_ = true;
  SDL_Window* window_;
  World world_;
  std::unique_ptr<Canvas<NodePolyDrawer> > drawer_;
};
}  // namespace diagrammar

#endif  // RUNTIME_SDL_SDL_INTERFACE_H_
