#ifndef UTILITY_TIMER_
#define UTILITY_TIMER_

#include <utility>
#include <chrono>

namespace diagrammar {

class Timer {
 public:
  Timer();
  double tick_time() const;
  double now() const;
  int64_t ticks() const;
  void Initialize();
  int32_t BeginNextFrame();

 private:
  typedef std::chrono::microseconds TimeUnit;
  int64_t accu_ticks_ = 0;
  // 1 / 60 s
  TimeUnit tick_time_ = TimeUnit(16667);
  TimeUnit accu_time_;
  TimeUnit elap_time_;
  std::chrono::high_resolution_clock::time_point init_time_;
  std::chrono::high_resolution_clock::time_point last_time_;
};
}

#endif