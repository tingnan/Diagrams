// Copyright 2015 Native Client Authors.

#ifndef RUNTIME_UTILITY_MAP_H_
#define RUNTIME_UTILITY_MAP_H_

#include <utility>
#include <cstddef>
#include <vector>
#include <queue>
#include <unordered_map>

namespace diagrammar {

// Utility container
// The class allows random access by index,
// and the address of every contained value is stable.
// Specialize std::allocator<std::pair<Key, Value>> and
// std::allocator<std::pair<Key, size_t>> for custom use
template <class Key, class Value,
          template <class _Key, class _Value, class... _OtherArgs>
          class MapType = std::unordered_map>
class IndexedMap {
 public:
  typedef std::pair<Key, Value> ValuePair;
  size_t size() const;
  // TODO(tingnan) maybe implement iterator and find
  bool contains(const Key& key) const;
  // Value type must have a default constructor
  Value& operator[](const Key& key);
  // Access by index (random access)
  const Value& get(size_t index) const;
  // Return count (0 or 1)
  size_t erase(const Key& key);

 private:
  std::vector<ValuePair> container_;
  MapType<Key, size_t> lookup_table_;
};

template <class Key, class Value, template <class _Key, class _Value,
                                            class... _OtherArgs> class MapType>
size_t IndexedMap<Key, Value, MapType>::size() const {
  return container_.size();
}

template <class Key, class Value, template <class _Key, class _Value,
                                            class... _OtherArgs> class MapType>
bool IndexedMap<Key, Value, MapType>::contains(const Key& key) const {
  return lookup_table_.find(key) != lookup_table_.end();
}

template <class Key, class Value, template <class _Key, class _Value,
                                            class... _OtherArgs> class MapType>
const Value& IndexedMap<Key, Value, MapType>::get(size_t index) const {
  return container_[index].second;
}

template <class Key, class Value, template <class _Key, class _Value,
                                            class... _OtherArgs> class MapType>
Value& IndexedMap<Key, Value, MapType>::operator[](const Key& key) {
  if (contains(key)) return container_[lookup_table_[key]].second;
  // Create a new element with the key;
  container_.emplace_back(std::make_pair(key, Value()));
  lookup_table_[key] = container_.size() - 1;
  return container_.back().second;
}

template <class Key, class Value, template <class _Key, class _Value,
                                            class... _OtherArgs> class MapType>
size_t IndexedMap<Key, Value, MapType>::erase(const Key& key) {
  if (contains(key)) {
    size_t idx = lookup_table_[key];
    size_t last_idx = container_.size() - 1;
    if (idx != last_idx) {
      // Swap with last element
      std::swap(container_[idx], container_[last_idx]);
      lookup_table_[key] = idx;
    }
    // Erase the last element
    container_.pop_back();
    lookup_table_.erase(key);
    return 1;
  }
  return 0;
}

// This is a bijection map (one to one only)
// No special allocators provided yet
template <class Key, class Value,
          template <class _Key, class _Value, class... _OtherArgs>
          class MapType = std::unordered_map>
class BiMap {
 public:
  bool contains_key(const Key& key) {
    return key_value_map_.find(key) != key_value_map_.end();
  }
  bool contains_value(const Value& val) {
    return value_key_map_.find(val) != value_key_map_.end();
  }
  // If key/value is not contained, the returned reference will be invalid
  // and may cause segfault
  Value& get_value(const Key& key) { key_value_map_.find(key)->second; }
  Key& get_key(const Value& val) { value_key_map_.find(val)->second; }
  size_t erase_by_key(const Key& key) {
    auto itr = key_value_map_.find(key);
    if (itr != key_value_map_.end()) {
      size_t num_erased = value_key_map_.erase(itr->second);
      assert(key_value_map_.erase(key) == num_erased);
      return num_erased;
    }
    return 0;
  }
  size_t erase_by_value(const Value& val) {
    auto itr = value_key_map_.find(val);
    if (itr != value_key_map_.end()) {
      size_t num_erased = key_value_map_.erase(itr->second);
      assert(value_key_map_.erase(val) == num_erased);
      return num_erased;
    }
    return 0;
  }
  void insert(std::pair<Key, Value> pair) {

  }
 private:
  MapType<Key, Value> key_value_map_;
  MapType<Value, Key> value_key_map_;
};

}  // namespace diagrammar

#endif  // RUNTIME_UTILITY_MAP_H_