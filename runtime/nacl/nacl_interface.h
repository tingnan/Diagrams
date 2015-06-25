// Copyright 2015 Native Client Authors.
#ifndef RUNTIME_NACL_NACL_INTERFACE_H_
#define RUNTIME_NACL_NACL_INTERFACE_H_
#include <ppapi/cpp/instance.h>
#include <ppapi/cpp/module.h>
#include <ppapi/cpp/graphics_3d.h>
#include <thread>
#include "physics/world.h"

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
}  // namespace diagrammar

namespace pp {

Module* CreateModule();

}  // namespace pp

#endif  // RUNTIME_NACL_NACL_INTERFACE_H_
