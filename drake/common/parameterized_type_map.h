#pragma once

#include <typeindex>
#include <typeinfo>
#include <unordered_map>

template <template <class> class ValueType>
class ParameterizedTypeMap {
 public:
  typedef std::unordered_map<std::type_index, std::shared_ptr<void>>
      InternalMap;

  template <class Key, class... Args>
  void emplace(Args&&... args) {
    map_[typeid(Key)] =
        std::make_shared<ValueType<Key>>(std::forward<Args>(args)...);
  }

  template <class Key>
  bool has_key() {
    return map_.count(typeid(Key)) > 0;
  }

  template <class Key>
  std::shared_ptr<ValueType<Key>> at() {
    return std::static_pointer_cast<ValueType<Key>>(map_.at(typeid(Key)));
  }

 private:
  InternalMap map_;
};
