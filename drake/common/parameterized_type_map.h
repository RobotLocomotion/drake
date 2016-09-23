#pragma once

#include <memory>
#include <typeindex>
#include <typeinfo>
#include <unordered_map>

/**
 * A map from a type Key to (a shared_ptr to) an instance of ValueType<Key>.
 */
template <template <class> class ValueType>
class ParameterizedTypeMap {
 public:
  typedef std::unordered_map<std::type_index, std::shared_ptr<void>>
      InternalMap;

  /**
   * Construct a new value type in place.
   *
   * @param args constructor arguments.
   */
  template <class Key, class... Args>
  void emplace(Args&&... args) {
    map_[typeid(Key)] =
        std::make_shared<ValueType<Key>>(std::forward<Args>(args)...);
  }

  /**
   * Returns true iff the key @tparam Key exists.
   *
   * @return whether key @tparam Key exists.
   */
  template <class Key>
  bool has_key() {
    return map_.count(typeid(Key)) > 0;
  }

  /**
   * Value access. Throws if key @tparam Key is not found.
   *
   * @return shared_ptr to the value type associated with @tparam Key.
   */
  template <class Key>
  std::shared_ptr<ValueType<Key>> get() {
    return std::static_pointer_cast<ValueType<Key>>(map_.at(typeid(Key)));
  }

 private:
  InternalMap map_;
};
