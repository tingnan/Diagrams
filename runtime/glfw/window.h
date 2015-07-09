#ifndef RENDER_WINDOW_
#define RENDER_WINDOW_

#include <GLFW/glfw3.h>

#include "gl/drawer.h"
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