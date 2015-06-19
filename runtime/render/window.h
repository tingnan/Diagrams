#ifndef _DIAGRAMMAR_GLAPP_
#define _DIAGRAMMAR_GLAPP_

#include "drawer.h"
#include <GLFW/glfw3.h>

#include "physics/world.h"

namespace diagrammar {
// the class manipulates the camera
class Window {
  // the tester
  // we will manage the window and mouse event here.
 private:
  // the main window
  GLFWwindow* glfw_window_;
  // the physics world!
  World world_;
  // the drawer
  Drawer drawer_;

  // We can implement a finite state machine
  // that tells each component where/when to go
  // e.g. aftere window initialization, we can go for world initialization
  // (including the physics engine)
  // then we can initialize the drawer and the camera
 private:
  void _InitializeWindow(int w, int h, const char* title);

 public:
  Window(int w, int h, const char* title);
  // not copyable;
  Window(const Window& other) = delete;
  ~Window() { glfwTerminate(); }
  void MainLoop();
};
}

#endif