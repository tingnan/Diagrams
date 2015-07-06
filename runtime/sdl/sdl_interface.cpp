// Copyright 2015 Native Client Authors.
#include <SDL2/SDL.h>
#include <SDL2/SDL_opengles2.h>
#include <iostream>
#include "sdl/sdl_interface.h"
#include "sdl/drawer.h"
#include "physics/world.h"
#include "utility/stl_memory.h"

namespace diagrammar {

SDLInterfaceOpenGL::SDLInterfaceOpenGL() {}

bool SDLInterfaceOpenGL::LoadFont() { return true; }

bool SDLInterfaceOpenGL::Init(int w, int h) {
  if (SDL_Init(SDL_INIT_VIDEO) < 0) {
    std::cerr << "error initialize\n";
    return false;
  }

  SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_ES);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 2);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 0);

  window_ = SDL_CreateWindow(nullptr, SDL_WINDOWPOS_UNDEFINED,
                             SDL_WINDOWPOS_UNDEFINED, w, h, SDL_WINDOW_OPENGL);
  if (SDL_GL_CreateContext(window_) == nullptr) {
    std::cerr << "error creating GL context\n";
    return false;
  }

  world_ = make_unique<World>();
  world_->InitializeWorldDescription("path_simple.json");
  world_->InitializePhysicsEngine();
  drawer_ = make_unique<Canvas<NodePathDrawer> >(0.0015);

  for (size_t i = 0; i < world_->GetNumNodes(); ++i) {
    drawer_->AddNode(world_->GetNodeByIndex(i));
  }

  SDL_StartTextInput();

  return true;
}

void SDLInterfaceOpenGL::HandleEvents() {
  SDL_Event event;
  while (SDL_PollEvent(&event) != 0) {
    if (event.type == SDL_QUIT) {
      app_running_ = false;
    }
  }
}

void SDLInterfaceOpenGL::Render() {
  while (app_running_) {
    HandleEvents();
    world_->Step();
    drawer_->Draw();
    SDL_GL_SwapWindow(window_);
  }
}

SDLInterfaceOpenGL::~SDLInterfaceOpenGL() {
  SDL_StopTextInput();
  SDL_DestroyWindow(window_);
  SDL_Quit();
}
}  // namespace diagrammar
