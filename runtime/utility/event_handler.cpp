// Copyright 2015 Native Client Authors.

#include <SDL2/SDL.h>

#include <json/json.h>
#include "utility/event_handler.h"

namespace {}  // namespace

const uint32_t kCustomEvent = SDL_RegisterEvents(1);

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

void PushEventToGlobalQueue(const Json::Value& evdata) {
  SDL_Event event;
  event.type = kCustomEvent;
  event.user.data1 = new Json::Value(evdata);
  SDL_PushEvent(&event);
}

}  // namespace diagrammar
