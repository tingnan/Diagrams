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

// Store all external node ids.
std::set<diagrammar::id_t> ext_node_ids;
// Each joint also connects two node, we also store their external ids;
std::map<diagrammar::id_t, std::pair<diagrammar::id_t, diagrammar::id_t>>
    ext_joint_ids;
// demo add and remove node from the world
std::unique_ptr<diagrammar::Node> test_add_remove_node(nullptr);
// demo add and remove joint from the world
std::unique_ptr<diagrammar::Joint> test_add_remove_joint_1(nullptr);
std::unique_ptr<diagrammar::Joint> test_add_remove_joint_2(nullptr);

// construct a world
std::unique_ptr<diagrammar::World> ParseWorld(const Json::Value& world) {
  diagrammar::World simulation_world;
  const Json::Value& node_obj = world["children"];
  Json::Value::const_iterator node_itr = node_obj.begin();
  for (; node_itr != node_obj.end(); ++node_itr) {
    std::unique_ptr<diagrammar::Node> node_ptr =
        diagrammar::ParseNode(*node_itr);
    // record the node id;
    ext_node_ids.insert(node_ptr->id);
    simulation_world.AddNode(std::move(node_ptr));
  }
  // can only parse the joint after we know all the nodes
  const Json::Value& joint_obj = world["joints"];
  Json::Value::const_iterator joint_itr = joint_obj.begin();
  for (; joint_itr != joint_obj.end(); ++joint_itr) {
    std::unique_ptr<diagrammar::Joint> joint_ptr =
        diagrammar::ParseJoint(*joint_itr);
    // record ext joint id and node id;
    ext_joint_ids[joint_ptr->id] =
        std::make_pair(joint_ptr->node_1, joint_ptr->node_2);
    simulation_world.AddJoint(std::move(joint_ptr));
  }
  return diagrammar::make_unique<diagrammar::World>(
      std::move(simulation_world));
}

diagrammar::GLProgram LoadSphereGLProgram() {
  std::string vert_shader = diagrammar::Stringify("sphere.vert");
  std::string frag_shader = diagrammar::Stringify("sphere.frag");
  diagrammar::GLProgram program;
  program.pid =
      diagrammar::CreateGLProgram(vert_shader.c_str(), frag_shader.c_str());
  program.u_mvp = glGetUniformLocation(program.pid, "u_mvp");
  program.color = glGetAttribLocation(program.pid, "color");
  program.vertex = glGetAttribLocation(program.pid, "vertex");
  program.resolution = glGetUniformLocation(program.pid, "resolution");
  return program;
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

  // world_.Read);
  world_ = ParseWorld((CreateJsonObject("path_simple.json")));
  world_->Start(World::EngineType::kLiquidFun);

  gl_program_ = LoadDefaultGLProgram();
  cameras_.resize(2);
  cameras_[0] =
      make_unique<Camera>(Vector3f(0, -50, 650), Vector3f(0, 50, 0),
                          Vector3f(0, 1, 0), M_PI / 2.0, 1.0, 10.0, 500000.0);

  cameras_[1] =
      make_unique<Camera>(Vector3f(0, 0, 800), Vector3f(0, 0, 0),
                          Vector3f(0, 1, 0), M_PI / 2.0, 1.0, 10.0, 500000.0);

  poly_debug_drawer_ = make_unique<NodeGroupDrawer<NodePolyDrawer>>(
      gl_program_, cameras_[0].get(), Vector2f(w, h));
  path_debug_drawer_ = make_unique<NodeGroupDrawer<NodePathDrawer>>(
      gl_program_, cameras_[0].get(), Vector2f(w, h));
  particle_drawer_ = make_unique<NodeGroupDrawer<SphereDrawer>>(
      LoadSphereGLProgram(), cameras_[0].get(), Vector2f(w, h));
  for (size_t i = 0; i < world_->GetNumNodes(); ++i) {
    auto& collision_shapes = world_->GetNodeByIndex(i)->collision_shapes;
    if (collision_shapes.size() == 1 &&
        collision_shapes[0]->shape_type == Shape2DType::kDisk) {
      particle_drawer_->AddNode(world_->GetNodeByIndex(i));
    }
    poly_debug_drawer_->AddNode(world_->GetNodeByIndex(i));
    path_debug_drawer_->AddNode(world_->GetNodeByIndex(i));
  }

  text_drawer_ = make_unique<TextDrawer>(font_);
  // use the fixed camera
  canvas_drawer_ = make_unique<CanvasDrawer>(cameras_[1].get(), Vector2f(w, h));

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
    world_->HandleMessage(event_message);
  }
}

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
    // Test Add/Remove feature of the system
    {
      if (message["key_code"] == "4" && message["key_pressed"] == true) {
        // Test remove a node by ext id;
        if (test_add_remove_node != nullptr) {
          const Node* tmp_ptr =
              world_->AddNode(std::move(test_add_remove_node));
          path_debug_drawer_->AddNode(tmp_ptr);
          poly_debug_drawer_->AddNode(tmp_ptr);
        } else {
          id_t test_id = 1;
          auto tmp_ptr = world_->RemoveNodeByExtID(test_id);
          if (tmp_ptr != nullptr) {
            path_debug_drawer_->RemoveNodeByID(tmp_ptr->id);
            poly_debug_drawer_->RemoveNodeByID(tmp_ptr->id);
            test_add_remove_node = std::move(tmp_ptr);
            test_add_remove_node->id = test_id;
          }
        }
        return true;
      }

      if (message["key_code"] == "Q" && message["key_pressed"] == true) {
        // Test remove/add joint by ext id;
        if (test_add_remove_joint_1 != nullptr) {
          world_->AddJoint(std::move(test_add_remove_joint_1));
        } else {
          id_t test_id = 0;
          auto tmp_ptr = world_->RemoveJointByExtID(test_id);
          if (tmp_ptr != nullptr) {
            test_add_remove_joint_1 = std::move(tmp_ptr);
            test_add_remove_joint_1->id = test_id;
            auto id_pair = ext_joint_ids[test_id];
            test_add_remove_joint_1->node_1 = id_pair.first;
            test_add_remove_joint_1->node_2 = id_pair.second;
          }
        }
        return true;
      }

      if (message["key_code"] == "E" && message["key_pressed"] == true) {
        // Test remove/add joint by ext id;
        if (test_add_remove_joint_2 != nullptr) {
          world_->AddJoint(std::move(test_add_remove_joint_2));
        } else {
          id_t test_id = 1;
          auto tmp_ptr = world_->RemoveJointByExtID(test_id);
          if (tmp_ptr != nullptr) {
            test_add_remove_joint_2 = std::move(tmp_ptr);
            test_add_remove_joint_2->id = test_id;
            auto id_pair = ext_joint_ids[test_id];
            test_add_remove_joint_2->node_1 = id_pair.first;
            test_add_remove_joint_2->node_2 = id_pair.second;
          }
        }
        return true;
      }
    }
  }

  if (message["type"] == "mouse_wheel") {
    float delta = message["y"].asFloat();
    cameras_[0]->Translate(Vector3f(0, 0, delta));
    return true;
  }

  if (message["type"] == "mouse_motion") {
    if (message["button_mask"].asUInt() & SDL_BUTTON_LMASK) {
      // We are moving the camera to the opposite position the mouse is moving
      float x_delta = -message["xrel"].asFloat();
      // In browser, y direction is reversed
      float y_delta = message["yrel"].asFloat();
      cameras_[0]->Translate(Vector3f(x_delta, y_delta, 0));
    }
  }

  return false;
}

void Application::RenderID() {
  for (size_t i = 0; i < world_->GetNumNodes(); ++i) {
    const Node* node = world_->GetNodeByIndex(i);
    if (true) {
      Vector3f pos = node->frame.GetTranslation();
      std::string label = std::to_string(node->id);
      label +=
          ":(" + std::to_string(pos(0)) + "," + std::to_string(pos(1)) + ")";
      text_drawer_->Draw(label, Vector2f(pos(0), pos(1)), cameras_[0].get());
    }
  }
}

void Application::Render() {
  while (app_running_) {
    HandleEvents();
    world_->Step();

    glClearColor(0.3, 0.3, 0.3, 1);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);

    auto curr_time = world_->now();
    if (true) canvas_drawer_->Draw(curr_time * 1e-3);
    if (draw_poly_) poly_debug_drawer_->Draw();
    if (draw_path_) path_debug_drawer_->Draw();
    if (draw_text_) RenderID();
    if (true) particle_drawer_->Draw();

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
