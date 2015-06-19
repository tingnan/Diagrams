#ifndef _DIAGRAMMAR_NACLINTERFACE_
#define _DIAGRAMMAR_NACLINTERFACE_

#include "physics/world.h"
#include "ppapi/cpp/instance.h"
#include "ppapi/cpp/module.h"
#include "ppapi/cpp/graphics_3d.h"
#include <thread>

namespace diagrammar {

class DiagrammarInterface : public pp::Instance {
  World world_;
  class NaClDrawer* drawer_;
  pp::Graphics3D context_;
  std::thread simulation_thread_;
  bool _InitGL(int32_t width, int32_t height);
  void _SimulationLoop();
  void _LaunchWorld();
  void _RenderLoop();
 public:
  explicit DiagrammarInterface(PP_Instance instance, pp::Module* module);
  virtual ~DiagrammarInterface();
  virtual void HandleMessage(const pp::Var& var_message);
  virtual void DidChangeView(const pp::View& view);
  // supposed to be called in background loop
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
