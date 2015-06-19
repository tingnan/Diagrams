#include <cstdlib>
#include <string>
#include <fstream>
#include <iostream>

#include "render/window.h"

int main() {
  diagrammar::Window myWindow(800, 800, "Test");
  myWindow.MainLoop();
  return 0;
}
