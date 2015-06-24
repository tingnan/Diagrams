#ifndef SDL_SDL_INTERFACE_
#define SDL_SDL_INTERFACE_
#include <memory>
class SDL_Window;
namespace diagrammar {
class SDLInterfaceOpenGL {
 public:
  SDLInterfaceOpenGL();
  bool Init(int, int);
  void Render();
  ~SDLInterfaceOpenGL();
 private:
  void HandleEvents();
  bool LoadFont();
  bool app_running_ = true;
  SDL_Window* window_;
  std::unique_ptr<class World> world_;
  std::unique_ptr<class NaClDrawer> drawer_;
};
}

#endif