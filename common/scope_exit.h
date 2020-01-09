#pragma once

#include <functional>
#include <utility>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"

namespace drake {

/// Helper class to create a scope exit guard -- an object that when destroyed
/// runs `func`.  This is useful to apply RAII to third-party code that only
/// supports manual acquire and release operations.
///
/// Example:
///
/// @code
/// void some_function() {
///   void* foo = ::malloc(10);
///   ScopeExit guard([foo]() {
///     ::free(foo);
///   });
///
///   // ...
///   if (condition) { throw std::runtime_error("..."); }
///   // ...
/// }
/// @endcode
///
/// Here, the allocation of `foo` will always be free'd no matter whether
/// `some_function` returns normally or via an exception.
class ScopeExit final {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ScopeExit);

  /// Creates a resource that will call `func` when destroyed.  Note that
  /// `func()` should not throw an exception, since it will typically be
  /// invoked during stack unwinding.
  explicit ScopeExit(std::function<void()> func)
      : func_(std::move(func)) {
    DRAKE_THROW_UNLESS(func_ != nullptr);
  }

  /// Invokes the `func` that was passed into the constructor, unless this has
  /// been disarmed.
  ~ScopeExit() {
    if (func_) {
      func_();
    }
  }

  /// Disarms this guard, so that the destructor has no effect.
  void Disarm() {
    func_ = nullptr;
  }

 private:
  std::function<void()> func_;
};

}  // namespace drake
