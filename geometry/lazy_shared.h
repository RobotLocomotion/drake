#pragma once

#include <memory>
#include <type_traits>

namespace drake {
namespace geometry {
namespace internal {

// LLVM libc++ doesn't support std::atomic<std::shared_ptr<T>>, so will need a
// different implementation; https://github.com/llvm/llvm-project/issues/99980.
#ifdef __cpp_lib_atomic_shared_ptr

// This is the implementation targeted at libstdc++ (GCC).

/* Wrapper that stores a T, with the ability to initalize it on first use in a
thread-safe way.

In case multiple threads call GetOrMake() concurrently, this class allows for
benign races where the factory function might be called twice but only one
winner is retained. (Implementing at-most-once instead of at-least-once would
be slower in the common case of no thread contention.)

Once this object has been initialized via call to GetOrMake(), any copies of it
(via copy-construction or copy-assignment) will share the same underlying
instance of `T` using reference counting (ala shared_ptr).

Moving from an initialized object (via move-construction or move-assignement)
will reset the moved-from object back to empty. */
template <typename T>
class LazyShared {
 public:
  /* Constructs an empty object. */
  LazyShared() = default;

  /* Copy construction shares ownership of existing data.
  See class overview for details. */
  LazyShared(const LazyShared& other) : data_(other.data_.load()) {}

  /* Move construction transfers ownership of existing data.
  See class overview for details. */
  LazyShared(LazyShared&& other) noexcept
      : data_(other.data_.exchange(nullptr)) {}

  /* Copy assignment shares ownership of existing data.
  See class overview for details. */
  LazyShared& operator=(const LazyShared& other) {
    if (this != &other) {
      data_.store(other.data_.load());
    }
    return *this;
  }

  /* Move assignment transfers ownership of existing data.
  See class overview for details. */
  LazyShared& operator=(LazyShared&& other) noexcept {
    if (this != &other) {
      data_.store(other.data_.exchange(nullptr));
    }
    return *this;
  }

  ~LazyShared() = default;

  /* If this object already has a value (e.g., if GetOrMake() has previously
  been called on this object), then returns the cached result. Otherwise, calls
  factory(), stores the result into this, and returns it. The `factory` function
  should return a `shared_ptr<const T>` (or something convertible to a
  `shared_ptr<const T>`), and should not have side-effects since it might be
  called twice. */
  template <typename Factory>
  const T& GetOrMake(Factory&& factory) const
    requires(std::is_invocable_r_v<std::shared_ptr<const T>, Factory>)
  {
    std::shared_ptr<const T> result = data_.load();
    if (result == nullptr) {
      std::shared_ptr<const T> candidate = factory();
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
  mutable std::atomic<std::shared_ptr<const T>> data_;
};

#else  // __cpp_lib_atomic_shared_ptr

// This is the implementation targeted at libc++ (LLVM).

template <typename T>
class LazyShared {
 public:
  LazyShared() = default;
  LazyShared(const LazyShared& other) : data_(std::atomic_load(&other.data_)) {}
  LazyShared(LazyShared&& other) noexcept
      : data_(std::atomic_exchange(&other.data_, std::shared_ptr<T>())) {}
  LazyShared& operator=(const LazyShared& other) {
    if (this != &other) {
      std::atomic_store(&data_, other.data_);
    }
    return *this;
  }
  LazyShared& operator=(LazyShared&& other) noexcept {
    if (this != &other) {
      std::atomic_store(
          &data_, std::atomic_exchange(&other.data_, std::shared_ptr<T>()));
    }
    return *this;
  }
  ~LazyShared() = default;
  template <typename Factory>
  const T& GetOrMake(Factory&& factory) {
    std::shared_ptr<T> result = std::atomic_load(&data_);
    if (result == nullptr) {
      std::shared_ptr<T> candidate = factory();
      if (std::atomic_compare_exchange_strong(&data_, &result, candidate)) {
        result = candidate;
      }
    }
    return *result;
  }

 private:
  mutable std::shared_ptr<T> data_;
};

#endif  // __cpp_lib_atomic_shared_ptr

}  // namespace internal
}  // namespace geometry
}  // namespace drake
