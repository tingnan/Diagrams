// Copyright 2015 Native Client Authors.
#include <SDL2/SDL_main.h>

#include <iostream>

#include "sdl/sdl_interface.h"
#include "physics/world.h"
#include "utility/world_parser.h"
#include "utility/stl_memory.h"

namespace diagrammar {

Application::Application() {}
Application::~Application() {
  SDL_StopTextInput();
  SDL_DestroyWindow(window_);
  SDL_Quit();
}


bool Application::LoadFont() { return true; }

bool Application::Init(int w, int h) {
  if (SDL_Init(SDL_INIT_VIDEO) < 0) {
    std::cerr << "error initialize\n";
    return false;
  }

  SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_ES);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 2);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 0);

  window_ = SDL_CreateWindow(nullptr, SDL_WINDOWPOS_UNDEFINED,
                             SDL_WINDOWPOS_UNDEFINED, w, h, SDL_WINDOW_OPENGL);
  gl_context_ = SDL_GL_CreateContext(window_);
  if (gl_context_ == nullptr) {
    std::cerr << "error creating GL context\n";
    return false;
  }

  world_.Read(CreateJsonObject("path_simple.json"));
  world_.Start();
  drawer_ = make_unique<Canvas<NodePolyDrawer> >(0.0015);

  for (size_t i = 0; i < world_.GetNumNodes(); ++i) {
    drawer_->AddNode(world_.GetNodeByIndex(i));
  }

  SDL_StartTextInput();
  return true;
}

void Application::HandleEvents() {
  SDL_Event event;
  while (SDL_PollEvent(&event) != 0) {
    switch (event.type) {
      case SDL_QUIT:
        app_running_ = false;
        break;
      default:
        break;
    }
  }
}

void Application::Render() {
  while (app_running_) {
    HandleEvents();
    world_.Step();
    drawer_->Draw();
    SDL_GL_SwapWindow(window_);
  }
}

}  // namespace diagrammar

int SDL_main (int argc, char *argv[]) {
  diagrammar::Application app;
  app.Init(800, 800);
  app.Render();
  return 0;
}