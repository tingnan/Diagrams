// Copyright 2015 Native Client Authors.
#include <GLES2/gl2.h>
#include <sys/mount.h>
#include <ppapi/gles2/gl2ext_ppapi.h>
#include <nacl_io/nacl_io.h>
#include <ppapi/cpp/var.h>
#include <ppapi/cpp/completion_callback.h>
#include <thread>
#include <iostream>
#include <string>
#include "nacl/nacl_interface.h"
#include "nacl/nacl_drawer.h"

namespace diagrammar {

void DiagrammarInterface::HandleMessage(const pp::Var& var_message) {
  // Ignore the message if it is not a string.
  if (!var_message.is_string()) return;

  // Get the string message and compare it to "hello".
  std::string message = var_message.AsString();
  if (message == "hello") {
    // If it matches, send our response back to JavaScript.
    pp::Var var_reply("running");
    PostMessage(var_reply);
  }
  if (message == "status?") {
    pp::Var var_reply(world_.now() - world_.simulation_time());
    PostMessage(var_reply);
  }
}

DiagrammarInterface::DiagrammarInterface(PP_Instance instance,
                                         pp::Module* module)
    : pp::Instance(instance) {
  // initialize the file system
  nacl_io_init_ppapi(instance, pp::Module::Get()->get_browser_interface());
  umount("/");
  mount("", "/http", "httpfs", 0, "");
}

void DiagrammarInterface::SimulationLoop() {
  world_.InitializeTimer();
  while (true) {
    world_.Step();
    drawer_->Draw();
    context_.SwapBuffers(pp::BlockUntilComplete());
  }
}

void DiagrammarInterface::RenderLoop() {}

void DiagrammarInterface::LaunchWorld() {
  glSetCurrentContextPPAPI(context_.pp_resource());
  world_.InitializeWorldDescription("/http/path_simple.json");
  world_.InitializePhysicsEngine();
  drawer_ = new NaClDrawer(world_);
  SimulationLoop();
}

bool DiagrammarInterface::InitGL(int32_t width, int32_t height) {
  if (!glInitializePPAPI(pp::Module::Get()->get_browser_interface())) {
    std::cerr << "Unable to initialize GL PPAPI!\n";
    return false;
  }
  if (context_.is_null()) {
    int32_t attrib_list[] = {
        PP_GRAPHICS3DATTRIB_ALPHA_SIZE, 8,
        PP_GRAPHICS3DATTRIB_DEPTH_SIZE, 24,
        PP_GRAPHICS3DATTRIB_WIDTH,      width,
        PP_GRAPHICS3DATTRIB_HEIGHT,     height,
        PP_GRAPHICS3DATTRIB_NONE,
    };
    context_ = pp::Graphics3D(this, attrib_list);

    assert(!context_.is_null());

    if (!BindGraphics(context_)) {
      std::cerr << "Unable to bind 3d context!\n";
      context_ = pp::Graphics3D();
      glSetCurrentContextPPAPI(0);
      return false;
    }
  }

  return true;
}

void DiagrammarInterface::DidChangeView(const pp::View& view) {
  int32_t width = view.GetRect().width();
  int32_t height = view.GetRect().height();
  std::cerr << "view changed\n";
  if (context_.is_null()) {
    if (!InitGL(width, height)) {
      return;
    }
    // only launch once (created with opengl context)
    simulation_thread_ = std::thread(&DiagrammarInterface::LaunchWorld, this);
  } else {
    int32_t result = context_.ResizeBuffers(width, height);
    if (result < 0) std::cerr << "Unable to resize\n";
  }
}

DiagrammarInterface::~DiagrammarInterface() {
  if (simulation_thread_.joinable()) {
    simulation_thread_.join();
  }
}

DiagrammarModule::DiagrammarModule() : pp::Module() {}
DiagrammarModule::~DiagrammarModule() {}

pp::Instance* DiagrammarModule::CreateInstance(PP_Instance instance) {
  return new DiagrammarInterface(instance, this);
}

}  // namespace diagrammar

pp::Module* pp::CreateModule() { return new diagrammar::DiagrammarModule(); }
