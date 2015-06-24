#ifndef _DIAGRAMMAR_GLAPP_
#define _DIAGRAMMAR_GLAPP_

#include "drawer.h"
#include <GLFW/glfw3.h>

#include "physics/world.h"

namespace diagrammar {
// the class manipulates the camera
class Window {
 public:
  Window(int w, int h, const char* title);
  // not copyable;
  Window(const Window& other) = delete;
  ~Window() { glfwTerminate(); }
  void MainLoop();
 private:
  void _InitializeWindow(int w, int h, const char* title);
  // the main window
  GLFWwindow* glfw_window_;
  // the physics world!
  World world_;
  // the drawer
  Drawer drawer_;
};
}

#endif