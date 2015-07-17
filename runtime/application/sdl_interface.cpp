// Copyright 2015 Native Client Authors.
#include <SDL2/SDL_main.h>

#include <iostream>

#include "application/sdl_interface.h"
#include "physics/world.h"
#include "utility/world_parser.h"
#include "utility/stl_memory.h"
#include "utility/event_handler.h"


namespace {
bool EmitSDLError(const char* message) {
  std::cerr << message << ": ";
  std::cerr << SDL_GetError() << std::endl;
  return false;
}

}  // namespace

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
    return EmitSDLError("error initialize SDL");
    return false;
  }
#ifdef __APPLE__
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
#else
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_ES);
#endif
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 2);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 0);

  window_ = SDL_CreateWindow(nullptr, SDL_WINDOWPOS_UNDEFINED,
                             SDL_WINDOWPOS_UNDEFINED, w, h, SDL_WINDOW_OPENGL);
  gl_context_ = SDL_GL_CreateContext(window_);
  if (gl_context_ == nullptr) {
    return EmitSDLError("error creating GL context");
  }
  
  if (TTF_Init() != 0) {
    return EmitSDLError("error initialize font system");
  }

  font_ = TTF_OpenFont("DejaVuSans.ttf", 12);
  if (font_ == nullptr) {
    std::string message;
    message += "error read font ";
    message += "DejaVuSans.ttf ";
    message += "at size" + std::to_string(12);
    return EmitSDLError(message.c_str());
  }

  world_.Read(CreateJsonObject("path_simple.json"));
  world_.Start();
  
  // 0.0015 is a scale, better to read from the input
  // e.g. 0.5 / max(world_.xspan(), world_.yspan());
  gl_program_ = LoadDefaultGLProgram();

  poly_drawers_ = make_unique<Canvas<NodePolyDrawer> >(gl_program_, 0.0015);
  for (size_t i = 0; i < world_.GetNumNodes(); ++i) {
    poly_drawers_->AddNode(world_.GetNodeByIndex(i));
  }

  path_drawers_ = make_unique<Canvas<NodePathDrawer> >(gl_program_, 0.0015);
  for (size_t i = 0; i < world_.GetNumNodes(); ++i) {
    path_drawers_->AddNode(world_.GetNodeByIndex(i));
  }

  text_drawer_ = make_unique<TextDrawer>(font_);

  // SDL_StartTextInput();
  return true;
}

void Application::HandleEvents() {
  SDL_Event event;
  while (SDL_PollEvent(&event) != 0) {
    Json::Value event_message;
    // our custom event, which is not a constexpr
    // the built in event types
    switch (event.type) {
      case SDL_QUIT:
        app_running_ = false;
        break;
      case SDL_MOUSEBUTTONDOWN:
      case SDL_MOUSEBUTTONUP:
        {
          // Please check SDL website for button enums
          event_message["type"] = "mouse_button";
          event_message["button"] = event.button.button;
          event_message["button_pressed"] = event.button.state == SDL_PRESSED;
          event_message["x"] = event.button.x;
          event_message["y"] = event.button.y;
        }
        break;
      case SDL_MOUSEMOTION:
        {
          // Please check SDL website for button mask enums
          event_message["type"] = "mouse_motion";
          event_message["button_mask"] = event.motion.state;
          event_message["x"] = event.motion.x;
          event_message["y"] = event.motion.y;
          event_message["xrel"] = event.motion.xrel;
          event_message["yrel"] = event.motion.yrel;
        }
        break;
      case SDL_KEYDOWN:
      case SDL_KEYUP:
        {
          event_message["type"] = "key";
          event_message["key_pressed"] = event.key.state == SDL_PRESSED;
          event_message["key_code"] = SDL_GetKeyName(event.key.keysym.sym);
        }
        break;
      default:
        break;
    }
    if (event_message.empty()) {
      continue;
    }
    // std::cout << event_message << std::endl;
    if (HandleMessage(event_message)) {
      continue;
    }
    world_.HandleMessage(event_message);
  }

}

bool Application::HandleMessage(const Json::Value& message) {
  // we define only a few basic key mappings here
  if (message["type"] == "key") {
    if (message["key_code"] == "1" && message["key_pressed"] == true) {
      draw_path_ = !draw_path_;
      return true;
    }

    if (message["key_code"] == "2" && message["key_pressed"] == true) {
      draw_poly_ = !draw_poly_;
      return true;
    }
  }

  return false;
}

void Application::Render() {
  while (app_running_) {
    HandleEvents();
    world_.Step();
    
    glClearColor(0.3, 0.3, 0.3, 1);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    if (draw_poly_)
      poly_drawers_->Draw();
    if (draw_path_)
      path_drawers_->Draw();
    text_drawer_->Draw("Hello World", gl_program_, Vector2f(800, 800), 0.003);
    SDL_GL_SwapWindow(window_);
  }
}

}  // namespace diagrammar

int main (int argc, char *argv[]) {
  diagrammar::Application app;
  if (!app.Init(800, 800)) {
    return 0;
  }
  app.Render();
  return 0;
}
