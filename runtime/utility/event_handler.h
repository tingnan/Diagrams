// Copyright 2015 Native Client Authors.
#ifndef RUNTIME_UTILITY_EVENTHANDLER_H_
#define RUNTIME_UTILITY_EVENTHANDLER_H_

#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>

namespace Json {
class Value;
}  // namespace Json

namespace diagrammar {

// the event queue template type
template <typename EventType, template <typename> class QueueType = std::queue>
class ThreadSafeQueue {
  std::mutex mutex_;
  std::condition_variable cvar_;
  QueueType<EventType> queue_;
  int max_size_ = 1000;

 public:
  void push(EventType data);
  EventType wait_and_pop();
  // potential of racing
  EventType try_pop();
};

extern const uint32_t kCustomEvent;
void PushEventToGlobalQueue(const Json::Value& evdata);

}  // namespace diagrammar

#endif  // RUNTIME_UTILITY_EVENTHANDLER_H_
