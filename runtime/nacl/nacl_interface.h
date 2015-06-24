#ifndef NACL_NACL_INTERFACE_
#define NACL_NACL_INTERFACE_

#include "physics/world.h"
#include "ppapi/cpp/instance.h"
#include "ppapi/cpp/module.h"
#include "ppapi/cpp/graphics_3d.h"
#include <thread>

namespace diagrammar {

class DiagrammarInterface : public pp::Instance {
 public:
  explicit DiagrammarInterface(PP_Instance instance, pp::Module* module);
  virtual ~DiagrammarInterface();
  virtual void HandleMessage(const pp::Var& var_message);
  virtual void DidChangeView(const pp::View& view);
 private:
  bool InitGL(int32_t width, int32_t height);
  void SimulationLoop();
  void LaunchWorld();
  void RenderLoop();
  World world_;
  class NaClDrawer* drawer_;
  pp::Graphics3D context_;
  std::thread simulation_thread_;
};

class DiagrammarModule : public pp::Module {
 public:
  DiagrammarModule();
  virtual ~DiagrammarModule();
  virtual ::pp::Instance* CreateInstance(PP_Instance instance);
};
}

namespace pp {

Module* CreateModule();

}  // namespace pp
#endif
