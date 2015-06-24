// Copyright 2015 Native Client Authors.
#include "utility/event_handler.h"

namespace diagrammar {

template <typename EventType, template <typename> class QueueType>
void ThreadSafeQueue<EventType, QueueType>::push(EventType data) {
  std::unique_lock<std::mutex> elock(mutex_);
  if (queue_.size() > max_size_) {
    // drop the incomming event
    return;
  }
  queue_.emplace(std::move(data));
  elock.unlock();
  cvar_.notify_one();
}

template <typename EventType, template <typename> class QueueType>
EventType ThreadSafeQueue<EventType, QueueType>::wait_and_pop() {
  std::unique_lock<std::mutex> elock(mutex_);
  cvar_.wait(elock, [this] { return !queue_.empty(); });
  EventType tmp = queue_.front();
  queue_.pop();
  return tmp;
}

template <typename EventType, template <typename> class QueueType>
EventType ThreadSafeQueue<EventType, QueueType>::try_pop() {
  std::unique_lock<std::mutex> elock(mutex_);
  EventType tmp;
  if (queue_.empty()) return tmp;
  tmp = queue_.front();
  queue_.pop();
  return tmp;
}

}  // namespace diagrammar
