#ifndef UTILITY_STL_MEMORY_
#define UTILITY_STL_MEMORY_
#include <memory>
// make_unique is only availble in c++14,
// we implement it here for c++11
namespace diagrammar {
template <typename T, typename... Args>
std::unique_ptr<T> make_unique(Args&&... args) {
  return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}
}
#endif