#include "sdl_interface.h"
#include <SDL2/SDL.h>
#include <GLES2/gl2.h>
#include <SDL2/SDL_opengl.h>
#include <iostream>
#include <physics/world.h>
#include "nacl_drawer.h"
#include <utility/stl_memory.h>

namespace diagrammar {
  
SDLInterfaceOpenGL::SDLInterfaceOpenGL() {
}

bool SDLInterfaceOpenGL::Init(int w, int h) {

  if (SDL_Init(SDL_INIT_VIDEO) < 0) {
    std::cerr << "error initialize\n";
    return false;
  }

  SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_ES);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 2);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 0);

  window_ = SDL_CreateWindow(nullptr, SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, w, h, SDL_WINDOW_OPENGL);
  if (SDL_GL_CreateContext(window_) == nullptr) {
    std::cerr << "error creating GL context\n";
    return false;
  }
  SDL_StartTextInput();


  world_ = make_unique<World>();
  world_->InitializeWorldDescription("path_simple.json");
  world_->InitializePhysicsEngine();
  drawer_ = make_unique<NaClDrawer>(*world_.get());
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
}