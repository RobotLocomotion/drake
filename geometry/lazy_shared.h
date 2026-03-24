#pragma once

#include <memory>

namespace drake {
namespace geometry {
namespace internal {

/* Wrapper that stores a T, with the ability to initalize it on first use in a
thread-safe way. This class allows for benign races where the factory function
might be called twice but only one winner is retained. */
template <typename T>
class LazyShared {
 public:
  /* Constructs an empty object. */
  LazyShared() = default;

  /* Copy construction shares ownership of existing data. */
  LazyShared(const LazyShared& other) : data_(other.data_.load()) {}

  /* Move construction transfers ownership of existing data. */
  LazyShared(LazyShared&& other) noexcept
      : data_(other.data_.exchange(nullptr)) {}

  /* Copy assignment shares ownership of existing data. */
  LazyShared& operator=(const LazyShared& other) {
    if (this != &other) {
      data_.store(other.data_.load());
    }
    return *this;
  }

  /* Move assignment transfers ownership of existing data. */
  LazyShared& operator=(LazyShared&& other) noexcept {
    if (this != &other) {
      data_.store(other.data_.exchange(nullptr));
    }
    return *this;
  }

  /* If this object already has a value (e.g., if GetOrMake() has previously
  been called on this object), then returns the cached result. Otherwise, calls
  factory(), stores the result into this, and returns it. The `factory` function
  should not have side-effects, since it might be called twice. */
  template <typename Factory>
  const T& GetOrMake(Factory&& factory) {
    std::shared_ptr<T> result = data_.load();
    if (result == nullptr) {
      std::shared_ptr<T> candidate = factory();
      if (data_.compare_exchange_strong(/* expected = */ result,
                                        /* desired = */ candidate)) {
        // The `candidate` was stored into `data_`; it's what we should return.
        result = candidate;
      } else {
        // The compare_exchange_strong overwrote `result` with the (non-null)
        // value computed by some other thread.
      }
    }
    return *result;
  }

 private:
  std::atomic<std::shared_ptr<T>> data_;
};

}  // namespace internal
}  // namespace geometry
}  // namespace drake
