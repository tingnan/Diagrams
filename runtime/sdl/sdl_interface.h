#ifndef _DIAGRAMMAR_SDL_INTERFACE_
#define _DIAGRAMMAR_SDL_INTERFACE_
#include <memory>
class SDL_Window;
namespace diagrammar {
class SDLInterfaceOpenGL {
SDL_Window* window_;
std::unique_ptr<class World> world_;
std::unique_ptr<class NaClDrawer> drawer_;
bool app_running_ = true;
void HandleEvents();
bool LoadFont();
public:
  SDLInterfaceOpenGL();
  bool Init(int, int);
  void Render();
  ~SDLInterfaceOpenGL();
};
}

#endif