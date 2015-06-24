#include "timer.h"
#include <iostream>

namespace {
const std::chrono::milliseconds k050ms = std::chrono::milliseconds(50);
const std::chrono::milliseconds k000ms = std::chrono::milliseconds(0);
}

namespace diagrammar {

Timer::Timer() {}
void Timer::Initialize() {
  // is the time monotonic?
  init_time_ = std::chrono::high_resolution_clock::now();
  last_time_ = std::chrono::high_resolution_clock::now();
  accu_time_ = std::chrono::microseconds::zero();
}

// this call takes less than 1 us to finish
int32_t Timer::BeginNextFrame() {
  std::chrono::high_resolution_clock::time_point curr_time =
      std::chrono::high_resolution_clock::now();
  elap_time_ = std::chrono::duration_cast<TimeUnit>(curr_time - last_time_);
  elap_time_ = elap_time_ > k000ms ? elap_time_ : k000ms;
  elap_time_ = elap_time_ < k050ms ? elap_time_ : tick_time_;
  int32_t num_ticks = 0;
  accu_time_ += elap_time_;
  while (accu_time_ >= tick_time_) {
    accu_time_ -= tick_time_;
    ++num_ticks;
  }
  last_time_ = curr_time;
  if (accu_ticks_ < INT64_MAX - num_ticks)
    accu_ticks_ += num_ticks;
  else
    accu_ticks_ = 0;
  return num_ticks;
}

// warning, must be in unit second
double Timer::tick_time() const {
  std::chrono::duration<float> fs = tick_time_;
  return fs.count();
}

double Timer::now() const {
  auto count = std::chrono::duration_cast<std::chrono::microseconds>(
                   std::chrono::high_resolution_clock::now() - init_time_)
                   .count();
  return double(count) * 0.001;
}

int64_t Timer::ticks() const { return accu_ticks_; }
}