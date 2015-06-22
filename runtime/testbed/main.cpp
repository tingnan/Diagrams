#include "sdl/sdl_interface.h"

int main() {
  diagrammar::SDLInterfaceOpenGL interface;
  interface.Init(800, 800);
  interface.Render();
  return 0;
}
