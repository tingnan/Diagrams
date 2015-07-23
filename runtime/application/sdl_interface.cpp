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
  TTF_CloseFont(font_);
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

  // Enable vsync
  SDL_GL_SetSwapInterval(1);

  if (TTF_Init() != 0) {
    return EmitSDLError("error initialize font system");
  }

  font_ = TTF_OpenFont("DejaVuSans.ttf", 14);
  if (font_ == nullptr) {
    std::string message;
    message += "error read font ";
    message += "DejaVuSans.ttf ";
    message += "at size" + std::to_string(14);
    return EmitSDLError(message.c_str());
  }

  world_.Read(CreateJsonObject("path_simple.json"));
  world_.Start();

  // 0.0015 is a scale, better to read from the input
  // e.g. 0.5 / max(world_.xspan(), world_.yspan());
  glViewport(0, 0, 800, 800);
  gl_program_ = LoadDefaultGLProgram();
  camera_.SetView(Vector3f(0, 0, 800), Vector3f(0, 0, 0), Vector3f(0, 1, 0));
  camera_.SetPerspective(M_PI / 2.0, 1, 10, 10000);

  poly_drawers_ = make_unique<Canvas<NodePolyDrawer> >(gl_program_, &camera_);
  for (size_t i = 0; i < world_.GetNumNodes(); ++i) {
    poly_drawers_->AddNode(world_.GetNodeByIndex(i));
  }

  path_drawers_ = make_unique<Canvas<NodePathDrawer> >(gl_program_, &camera_);
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
    switch (event.type) {
      case SDL_QUIT:
        app_running_ = false;
        break;
      case SDL_MOUSEBUTTONDOWN:
      case SDL_MOUSEBUTTONUP: {
        // Please check SDL website for button enums.
        event_message["type"] = "mouse_button";
        event_message["button"] = event.button.button;
        event_message["button_pressed"] = event.button.state == SDL_PRESSED;
        event_message["x"] = event.button.x;
        event_message["y"] = event.button.y;
      } break;
      case SDL_MOUSEMOTION: {
        // Please check SDL website for button mask enums.
        event_message["type"] = "mouse_motion";
        event_message["button_mask"] = event.motion.state;
        event_message["x"] = event.motion.x;
        event_message["y"] = event.motion.y;
        event_message["xrel"] = event.motion.xrel;
        event_message["yrel"] = event.motion.yrel;
      } break;
      case SDL_MOUSEWHEEL: {
        // Please check SDL website for event detail.
        event_message["type"] = "mouse_wheel";
        event_message["x"] = event.wheel.x;
        event_message["y"] = event.wheel.y;
      } break;

      case SDL_KEYDOWN:
      case SDL_KEYUP: {
        event_message["type"] = "key";
        event_message["key_pressed"] = event.key.state == SDL_PRESSED;
        // Warning, there
        event_message["key_code"] = SDL_GetKeyName(event.key.keysym.sym);
      } break;
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

std::unique_ptr<Node> test_ptr(nullptr);
bool Application::HandleMessage(const Json::Value& message) {
  // We define only a few basic key mappings here.
  if (message["type"] == "key") {
    if (message["key_code"] == "1" && message["key_pressed"] == true) {
      draw_path_ = !draw_path_;
      return true;
    }

    if (message["key_code"] == "2" && message["key_pressed"] == true) {
      draw_poly_ = !draw_poly_;
      return true;
    }

    if (message["key_code"] == "3" && message["key_pressed"] == true) {
      draw_text_ = !draw_text_;
      return true;
    }

    if (message["key_code"] == "4" && message["key_pressed"] == true) {
      // Test remove an node by ext id;
      auto tmp_ptr = world_.RemoveNodeByExtID(1);
      if (tmp_ptr != nullptr) {
        path_drawers_->RemoveNodeByID(tmp_ptr->id);
        poly_drawers_->RemoveNodeByID(tmp_ptr->id);
        test_ptr = std::move(tmp_ptr);
        test_ptr->id = 1;
      }
    }

    if (message["key_code"] == "5" && message["key_pressed"] == true) {
      // Test add the moved node back
      if (test_ptr != nullptr) {
        const Node* tmp_ptr = world_.AddNode(std::move(test_ptr));
        path_drawers_->AddNode(tmp_ptr);
        poly_drawers_->AddNode(tmp_ptr);
      }
    }
  }

  if (message["type"] == "mouse_wheel") {
    float delta = message["y"].asFloat();
    camera_.Translate(Vector3f(0, 0, delta));
    return true;
  }

  if (message["type"] == "mouse_motion") {
    if (message["button_mask"].asUInt() & SDL_BUTTON_LMASK) {
      float x_delta = message["xrel"].asFloat();
      float y_delta = message["yrel"].asFloat();
      camera_.Translate(Vector3f(x_delta, y_delta, 0));
    }
  }

  return false;
}

void Application::RenderID() {
  for (size_t i = 0; i < world_.GetNumNodes(); ++i) {
    const Node* node = world_.GetNodeByIndex(i);
    if (true) {
      Vector2f pos = node->frame.GetTranslation();
      std::string label = std::to_string(node->id);
      label +=
          ":(" + std::to_string(pos(0)) + "," + std::to_string(pos(1)) + ")";
      text_drawer_->Draw(label, pos, gl_program_, &camera_);
    }
  }
}

void Application::Render() {
  while (app_running_) {
    HandleEvents();
    world_.Step();

    glClearColor(0.3, 0.3, 0.3, 1);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    if (draw_poly_) poly_drawers_->Draw();
    if (draw_path_) path_drawers_->Draw();
    if (draw_text_) RenderID();
    SDL_GL_SwapWindow(window_);
  }
}

}  // namespace diagrammar

int main(int argc, char* argv[]) {
  diagrammar::Application app;
  if (!app.Init(800, 800)) {
    return 0;
  }
  app.Render();
  return 0;
}
