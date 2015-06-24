// Copyright 2015 Native Client Authors.
#ifndef RUNTIME_UTILITY_TIMER_H_
#define RUNTIME_UTILITY_TIMER_H_

#include <utility>
#include <chrono>

namespace diagrammar {

// a class used to synchronize the real world time with simulation time
class Timer {
 public:
  Timer();
  // get the time step, in second
  double tick_time() const;
  // get the elpased time since timer initialized, in microsecond
  double now() const;
  // accumulated ticks
  int64_t accumulated_ticks() const;
  // initialize the timer
  void Initialize();
  // compute the number of ticks needed since last call
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
}  // namespace diagrammar

#endif  // RUNTIME_UTILITY_TIMER_H_
