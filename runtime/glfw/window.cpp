#include <cstdlib>
#include <iostream>

#include "window.h"

namespace diagrammar {
Window::Window(int w, int h, const char* title) : drawer_(world_) {
  // first we use GLFW to create a OpenGL context
  _InitializeWindow(w, h, title);

  world_.LoadWorld("path_simple.json");
  world_.InitializePhysicsEngine();
  drawer_.GenPathBuffers();
  drawer_.GenPolyBuffers();
  drawer_.LoadShaders();
}

void Window::_InitializeWindow(int w, int h, const char* title) {
  if (!glfwInit()) exit(-1);

  glfwWindowHint(GLFW_SAMPLES, 8);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 4);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

  glfw_window_ = glfwCreateWindow(w, h, "Diagrammar", nullptr, nullptr);

  if (!glfw_window_) {
    glfwTerminate();
    exit(-1);
  }
  glfwMakeContextCurrent(glfw_window_);

  // now we have created the opengl context, initialize GL

  GLint major = 0;
  glGetIntegerv(GL_MAJOR_VERSION, &major);

  glewExperimental = GL_TRUE;
  GLenum glewerr = glewInit();
  if (glewInit() != GLEW_OK) {
    std::cout << "OpenGL version: " << major << std::endl;
    std::cout << "Failed to initialize glew: " << glewGetErrorString(glewerr)
              << std::endl;
    glfwTerminate();
    exit(-1);
  }
}

void Window::MainLoop() {
  // world_.InitializeTimer();
  while (!glfwWindowShouldClose(glfw_window_)) {
    // stepping code goes here but we can also create a separate thread that
    // steps the world

    // mTimer.BeginNextFrame();
    glfwPollEvents();
    world_.Step();
    drawer_.Draw();
    glfwSwapBuffers(glfw_window_);
  }
}
}
