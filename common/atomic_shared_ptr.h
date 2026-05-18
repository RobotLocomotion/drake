#pragma once

#include <memory>
#include <type_traits>
#include <utility>

#include "drake/common/drake_copyable.h"

namespace drake {
namespace internal {

// LLVM libc++ doesn't support std::atomic<std::shared_ptr<T>>, so will need a
// different implementation; https://github.com/llvm/llvm-project/issues/99980.
#ifdef __cpp_lib_atomic_shared_ptr

// This is the implementation targeted at libstdc++ (GCC).

template <typename T>
using atomic_shared_ptr = std::atomic<std::shared_ptr<T>>;

#else  // __cpp_lib_atomic_shared_ptr

// This is the implementation targeted at libc++ (LLVM).

/* Polyfill implementation of std::atomic<std::shared_ptr<T>>> based on the
deprecated free-function API. Note that this implementation will not perform
as well as the newer std::atomic<std::shared_ptr<T>> because the deprecated
free-function API typically uses a fixed-size global hash table of mutexes.

See https://en.cppreference.com/cpp/memory/shared_ptr/atomic2 and
https://en.cppreference.com/cpp/memory/shared_ptr/atomic. */
template <typename T>
class atomic_shared_ptr {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(atomic_shared_ptr);

  using value_type = std::shared_ptr<T>;

  constexpr atomic_shared_ptr() noexcept = default;

  // NOLINTNEXTLINE(runtime/explicit) To match standard API.
  constexpr atomic_shared_ptr(std::nullptr_t) noexcept : atomic_shared_ptr() {}

  // NOLINTNEXTLINE(runtime/explicit) To match standard API.
  atomic_shared_ptr(std::shared_ptr<T> desired) noexcept
      : ptr_(std::move(desired)) {}

  void operator=(std::shared_ptr<T> desired) noexcept {
    this->store(std::move(desired));
  }

  bool is_lock_free() const noexcept { return std::atomic_is_lock_free(&ptr_); }

  void store(std::shared_ptr<T> desired,
             std::memory_order order = std::memory_order_seq_cst) noexcept {
    std::atomic_store_explicit(&ptr_, std::move(desired), order);
  }

  std::shared_ptr<T> load(
      std::memory_order order = std::memory_order_seq_cst) const noexcept {
    return std::atomic_load_explicit(&ptr_, order);
  }

  operator std::shared_ptr<T>() const noexcept { return load(); }

  std::shared_ptr<T> exchange(
      std::shared_ptr<T> desired,
      std::memory_order order = std::memory_order_seq_cst) noexcept {
    return std::atomic_exchange_explicit(&ptr_, std::move(desired), order);
  }

  bool compare_exchange_strong(std::shared_ptr<T>& expected,
                               std::shared_ptr<T> desired,
                               std::memory_order success,
                               std::memory_order failure) noexcept {
    return std::atomic_compare_exchange_strong_explicit(
        &ptr_, &expected, std::move(desired), success, failure);
  }

  bool compare_exchange_weak(std::shared_ptr<T>& expected,
                             std::shared_ptr<T> desired,
                             std::memory_order success,
                             std::memory_order failure) noexcept {
    return std::atomic_compare_exchange_weak_explicit(
        &ptr_, &expected, std::move(desired), success, failure);
  }

  bool compare_exchange_strong(
      std::shared_ptr<T>& expected, std::shared_ptr<T> desired,
      std::memory_order order = std::memory_order_seq_cst) noexcept {
    const std::memory_order fail_order =
        (order == std::memory_order_acq_rel)   ? std::memory_order_acquire
        : (order == std::memory_order_release) ? std::memory_order_relaxed
                                               : order;
    return this->compare_exchange_strong(expected, std::move(desired), order,
                                         fail_order);
  }

  bool compare_exchange_weak(
      std::shared_ptr<T>& expected, std::shared_ptr<T> desired,
      std::memory_order order = std::memory_order_seq_cst) noexcept {
    const std::memory_order fail_order =
        (order == std::memory_order_acq_rel)   ? std::memory_order_acquire
        : (order == std::memory_order_release) ? std::memory_order_relaxed
                                               : order;
    return this->compare_exchange_weak(expected, std::move(desired), order,
                                       fail_order);
  }

  // We are unable to implement this easily, but we don't need it.
  void wait(std::shared_ptr<T> old,
            std::memory_order order =
                std::memory_order_seq_cst) const noexcept = delete;

  // We are unable to implement this easily, but we don't need it.
  void notify_one() noexcept = delete;

  // We are unable to implement this easily, but we don't need it.
  void notify_all() noexcept = delete;

  static constexpr bool is_always_lock_free = false;

 private:
  std::shared_ptr<T> ptr_;
};

#endif  // __cpp_lib_atomic_shared_ptr

}  // namespace internal
}  // namespace drake
