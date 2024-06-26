#pragma once

#include <atomic>
#include <utility>

#include "drake/common/hwy_dynamic.h"

// This file should only ever be included from `*.cc` implementation files,
// and we always want its code to be private to that file, so we'll use an
// anonymous namespace in a header file, for simplicity.
namespace {  // NOLINT(build/namespaces)

/* LateBoundFunction is a small wrapper class around a C-style raw function
pointer. (It doesn't support std::function objects.)

In a given process, the first time our Call() function is called, it will
latch-initialize the wrapped function pointer as follows:
- call the ChooseFunctor to select a which function pointer to use,
- memorize that answer for next time;
- call the selected function pointer and return its result.

After the selected function pointer is memorized, future calls to Call() will
invoke it efficiently (with no conditional branching).

Note that the memorization is thread-safe, but not limited to at-most-once
semantics. Multiple threads might concurrently select and memorize the choice,
the ChooseFunctor might be called multiple times.

In support of unit testing, the selection can be unlatched via Reset().

@tparam ChooseFunctor thread-safe functor that selects the function pointer to
be wrapped. */
template <typename ChooseFunctor>
class LateBoundFunction {
 public:
  /* A function pointer type, determined by what the ChooseFunctor returns. */
  using FunctionPointer = decltype(ChooseFunctor()());

  /* Calls this trampoline. The first time we're called, we'll choose the best
  target function and memorize it; subsequent calls will directly call into the
  chosen target without any conditional checks. */
  template <typename... Args>
  __attribute__((always_inline)) static decltype(auto) Call(Args... args) {
    auto impl = function_.load(std::memory_order::relaxed);
    return impl(std::forward<Args>(args)...);
  }

  /* (For testing only) Clears the latched target detection. This allows for
  testing multiple different targets from the same test program. This function
  is NOT thread-safe; only use this in single-threaded tests. */
  __attribute__((cold)) static void Reset() {
    // The memory order here doesn't really matter; this must only ever be
    // called in a single-threaded context. Anyway we'll use still use relaxed
    // to match the rest of this file.
    function_.store(&LateBoundFunction::ChooseThenCall,
                    std::memory_order::relaxed);
  }

 private:
  template <typename... Args>
  __attribute__((cold)) static decltype(auto) ChooseThenCall(Args... args) {
    drake::internal::HwyDynamicRegisterResetFunction(&Reset);
    hwy::GetChosenTarget().Update(hwy::SupportedTargets());
    auto impl = ChooseFunctor()();
    function_.store(impl, std::memory_order::relaxed);
    return impl(args...);
  }

  // Static globals must be trivially destructible.
  static_assert(std::is_trivially_destructible_v<std::atomic<FunctionPointer>>);

  // All operations on this pointer must use memory_order::relaxed, which is
  // zero-cost on the platforms we care about. Note that the default value for
  // this variable is provided out-of-line below; it does NOT default to null.
  static std::atomic<FunctionPointer> function_;
};

template <typename ChooseFunctor>
std::atomic<decltype(ChooseFunctor()())>
    LateBoundFunction<ChooseFunctor>::function_ =
        &LateBoundFunction::ChooseThenCall;

}  // namespace
