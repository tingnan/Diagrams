// Copyright 2015 Native Client Authors.
#include <SDL2/SDL_main.h>

#include <iostream>
#include <fstream>

#include <sys/mount.h>

#include "application/sdl_interface.h"
#include "physics/world.h"
#include "utility/world_parser.h"
#include "utility/stl_memory.h"
#include "utility/event_handler.h"

namespace {

const char* kResourceFolderName = "resources";
const char* kFontFileName = "DroidSans.ttfd";
const char* kDemoFileName = "path_simple.jsond";

const char kFragShaderSource[] =
    "precision mediump float;\n"
    "varying vec4 v_color;\n"
    "void main() {\n"
    "  gl_FragColor = v_color;\n"
    "}\n";

const char kVertShaderSource[] =
    "uniform mat4 u_mvp;\n"
    "attribute vec4 vertex;\n"
    "attribute vec4 normal;\n"
    "attribute vec4 color;\n"
    "varying vec4 v_color;\n"
    "void main() {\n"
    "  gl_Position = u_mvp * vec4(vertex.xyz, 1.0);\n"
    "  v_color = vec4(color.xyz, 1.0);\n"
    "}\n";

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
  std::string resource_folder_name = std::string(kResourceFolderName) + "/";
  std::string vert_shader_name = resource_folder_name + "sphere.vert";
  std::string frag_shader_name = resource_folder_name + "sphere.frag";
  std::string vert_shader = diagrammar::Stringify(vert_shader_name.c_str());
  std::string frag_shader = diagrammar::Stringify(frag_shader_name.c_str());
  diagrammar::GLProgram program;
  program.pid =
      diagrammar::CreateGLProgram(vert_shader.c_str(), frag_shader.c_str());
  program.u_mvp = glGetUniformLocation(program.pid, "u_mvp");
  program.color = glGetAttribLocation(program.pid, "color");
  program.vertex = glGetAttribLocation(program.pid, "vertex");
  return program;
}

diagrammar::GLProgram LoadMesh3DGLProgram() {
  std::string resource_folder_name = std::string(kResourceFolderName) + "/";
  std::string vert_shader_name = resource_folder_name + "mesh3d.vert";
  std::string frag_shader_name = resource_folder_name + "mesh3d.frag";
  std::string vert_shader = diagrammar::Stringify(vert_shader_name.c_str());
  std::string frag_shader = diagrammar::Stringify(frag_shader_name.c_str());
  diagrammar::GLProgram program;
  program.pid =
      diagrammar::CreateGLProgram(vert_shader.c_str(), frag_shader.c_str());
  program.u_mvp = glGetUniformLocation(program.pid, "u_mvp");
  program.color = glGetAttribLocation(program.pid, "color");
  program.vertex = glGetAttribLocation(program.pid, "vertex");
  program.normal = glGetAttribLocation(program.pid, "normal");
  program.resolution = glGetUniformLocation(program.pid, "resolution");
  return program;
}

diagrammar::GLProgram LoadSimpleGLProgram() {
  diagrammar::GLProgram program;
  program.pid =
      diagrammar::CreateGLProgram(kVertShaderSource, kFragShaderSource);
  program.u_mvp = glGetUniformLocation(program.pid, "u_mvp");
  program.color = glGetAttribLocation(program.pid, "color");
  program.normal = glGetAttribLocation(program.pid, "normal");
  program.vertex = glGetAttribLocation(program.pid, "vertex");
  return program;
}

diagrammar::GLProgram LoadBackgroundGLProgram() {
  diagrammar::GLProgram program;
  std::string resource_folder_name = std::string(kResourceFolderName) + "/";
  std::string vert_shader_name = resource_folder_name + "bgshader.vert";
  std::string frag_shader_name = resource_folder_name + "bgshader.frag";
  std::string vert_shader = diagrammar::Stringify(vert_shader_name.c_str());
  std::string frag_shader = diagrammar::Stringify(frag_shader_name.c_str());
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

  std::string resource_folder_name = std::string(kResourceFolderName) + "/";
  std::string font_file_name = resource_folder_name + kFontFileName;
  font_ = TTF_OpenFont(font_file_name.c_str(), 14);
  if (font_ == nullptr) {
    std::string message;
    message += "error read font ";
    message += kFontFileName;
    message += "at size " + std::to_string(14);
    return EmitSDLError(message.c_str());
  }
  std::string demo_file_name = resource_folder_name + kDemoFileName;
  world_ = ParseWorld((CreateJsonObject(demo_file_name.c_str())));
  world_->Start(World::EngineType::kLiquidFun);

  // We have two camera setup: the first one for viewing the board and the
  // second one for background. The second camera does not move.
  cameras_.resize(2);
  cameras_[0] =
      make_unique<Camera>(Vector3f(0, -50, 650), Vector3f(0, 50, 0),
                          Vector3f(0, 1, 0), M_PI / 2.0, 1.0, 10.0, 500000.0);

  cameras_[1] =
      make_unique<Camera>(Vector3f(0, 0, 800), Vector3f(0, 0, 0),
                          Vector3f(0, 1, 0), M_PI / 2.0, 1.0, 10.0, 500000.0);

  poly_drawer_ = make_unique<NodeGroupDrawer<NodeBuldgedDrawer>>(
      LoadMesh3DGLProgram(), cameras_[0].get(), Vector2f(w, h));
  path_drawer_ = make_unique<NodeGroupDrawer<NodePathDrawer>>(
      LoadSimpleGLProgram(), cameras_[0].get(), Vector2f(w, h));
  particle_drawer_ = make_unique<NodeGroupDrawer<SphereDrawer>>(
      LoadSphereGLProgram(), cameras_[0].get(), Vector2f(w, h));
  // Add Node to each group drawer.
  for (size_t i = 0; i < world_->GetNumNodes(); ++i) {
    auto& collision_shapes = world_->GetNodeByIndex(i)->collision_shapes;
    if (collision_shapes.size() == 1 &&
        collision_shapes[0]->shape_type == Shape2DType::kDisk) {
      particle_drawer_->AddNode(world_->GetNodeByIndex(i));
    } else {
      poly_drawer_->AddNode(world_->GetNodeByIndex(i));
    }
    path_drawer_->AddNode(world_->GetNodeByIndex(i));
  }

  text_drawer_ = make_unique<TextDrawer>(font_);
  // Use the fixed camera for background drawing.
  canvas_drawer_ = make_unique<CanvasDrawer>(LoadBackgroundGLProgram(),
                                             cameras_[1].get(), Vector2f(w, h));
  return true;
}

void Application::WindowEvent(SDL_WindowEvent event) {
  if (event.event == SDL_WINDOWEVENT_RESIZED) {
  }
}

void Application::HandleEvents() {
  SDL_Event event;
  while (SDL_PollEvent(&event) != 0) {
    Json::Value event_message;
    switch (event.type) {
      case SDL_QUIT:
        app_running_ = false;
        break;
      case SDL_WINDOWEVENT: {
        WindowEvent(event.window);
      }
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
          path_drawer_->AddNode(tmp_ptr);
          poly_drawer_->AddNode(tmp_ptr);
        } else {
          id_t test_id = 1;
          auto tmp_ptr = world_->RemoveNodeByExtID(test_id);
          if (tmp_ptr != nullptr) {
            path_drawer_->RemoveNodeByID(tmp_ptr->id);
            poly_drawer_->RemoveNodeByID(tmp_ptr->id);
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

    // Cast time into seconds.
    auto curr_time = world_->now() * 1e-3;

    if (true) canvas_drawer_->Draw(curr_time);
    if (draw_poly_) {
      poly_drawer_->Draw(curr_time);
      particle_drawer_->Draw(curr_time);
    }
    if (draw_path_) path_drawer_->Draw(curr_time);
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
