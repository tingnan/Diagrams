#ifndef RUNTIME_UTILITY_STL_MEMORY_H_
#define RUNTIME_UTILITY_STL_MEMORY_H_
#include <memory>
// make_unique is only availble in c++14,
// we implement it here for c++11
namespace diagrammar {
template <typename T, typename... Args>
std::unique_ptr<T> make_unique(Args&&... args) {
  return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}
}  // namespace diagrammar
#endif  // RUNTIME_UTILITY_STL_MEMORY_H_
