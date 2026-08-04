#pragma once

#include <memory>
#include <type_traits>
#include <utility>

#include "drake/common/atomic_shared_ptr.h"
#include "drake/common/drake_assert.h"

namespace drake {
namespace geometry {
namespace internal {

/* Wrapper that stores a T, with the ability to initalize it on first use ("lazy
evaluation") in a thread-safe way.

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
  been called on this object), then returns the stored result. Otherwise, calls
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
      DRAKE_DEMAND(candidate != nullptr);
      if (data_.compare_exchange_strong(/* expected = */ result,
                                        /* desired = */ candidate)) {
        // The `candidate` was stored into `data_`; it's what we should return.
        result = std::move(candidate);
      } else {
        // The compare_exchange_strong overwrote `result` with the (non-null)
        // value computed by some other thread.
      }
    }
    return *result;
  }

 private:
  mutable drake::internal::atomic_shared_ptr<const T> data_;
};

}  // namespace internal
}  // namespace geometry
}  // namespace drake
