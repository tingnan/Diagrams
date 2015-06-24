#ifndef UTILITY_EVENTHANDLER_
#define UTILITY_EVENTHANDLER_
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>

namespace diagrammar {
// the event queue template type
template<typename EventType, template<typename> class QueueType = std::queue >
class ThreadSafeQueue {
  std::mutex mutex_;
  std::condition_variable cvar_;
  QueueType<EventType> queue_;
  int max_size_ = 1000;
 public:
  void push(EventType data);
  EventType wait_and_pop();
  EventType try_pop();
};

}

#endif