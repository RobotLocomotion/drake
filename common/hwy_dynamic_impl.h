#pragma once

#include <atomic>
#include <utility>

#include "drake/common/cpu_capabilities.h"
#include "drake/common/hwy_dynamic.h"

// This file should only ever be included from `*.cc` implementation files,
// and we always want its code to be private to that file, so we'll use an
// anonymous namespace in a header file, for simplicity.
namespace {  // NOLINT(build/namespaces_headers)

/* LateBoundFunction is a small wrapper class around a C-style raw function
pointer. (It doesn't support std::function objects.)

In a given process, the first time our Call() function is called, it will
latch-initialize the wrapped function pointer as follows:
- call the ChooseFunctor to select which function pointer to use,
- memorize that answer for next time;
- call the selected function pointer and return its result.

After the selected function pointer is memorized, future calls to Call() will
invoke it efficiently (with no conditional branching).

Note that the memorization is thread-safe, but not limited to at-most-once
semantics. Multiple threads might concurrently select and memorize the choice,
the ChooseFunctor might be called multiple times.

In support of unit testing, selections can be unlatched via HwyDynamicReset()
in "drake/common/hwy_dynamic.h".

@tparam ChooseFunctor thread-safe functor that selects the function pointer to
be wrapped. */
template <typename ChooseFunctor>
class LateBoundFunction {
 public:
  /* A function pointer type, determined by what the ChooseFunctor returns. */
  using FunctionPointer = decltype(ChooseFunctor()());

  /* Calls this trampoline. The first time we're called, we'll choose the best
  target function and memorize it; subsequent calls will directly call into the
  chosen target without any conditional checks. Caveat: our memorization does
  not use the sequential consistency flavor of atomics, so the "first time" here
  actually means "first time on this thread"; if other threads happen to be
  bootstrapping at the same time, then in the extreme ChooseThenCall() could
  end up being run once per CPU core. */
  template <typename... Args>
  __attribute__((always_inline)) static decltype(auto) Call(Args... args) {
    // We use "consume" memory order here because we only care about memory
    // consistency as viewed from the current thread, not other threads. It's
    // safe for multiple threads to end up calling ChooseThenCall() in case they
    // all happen to run at once, in which case they will all store identical
    // values back into `function_`.
    auto impl = function_.load(std::memory_order::consume);
    return impl(std::forward<Args>(args)...);
  }

 private:
  template <typename... Args>
  __attribute__((cold)) static decltype(auto) ChooseThenCall(Args... args) {
    // Add our reset handler into the global registry. (This only registers the
    // function, it doesn't call it.)
    drake::internal::HwyDynamicRegisterResetFunction(&Reset);
    // Force highway to select a CPU target, if it hasn't already.
    if (auto& target = hwy::GetChosenTarget(); !target.IsInitialized()) {
      target.Update(hwy::SupportedTargets() &
                    drake::internal::GetHighwayAllowedTargetMask());
    }
    // Retrieve the CPU-specific function pointer from the table of pointers.
    auto impl = ChooseFunctor()();
    // Memorize the selected pointer for next time (or until a Reset()). We use
    // "release" memory order here because we only care about memory consistency
    // as viewed from the current thread.
    function_.store(impl, std::memory_order::release);
    // Call the actual function we want and return its result.
    return impl(args...);
  }

  /* (For testing only) Clears the latched target detection. This allows for
  testing multiple different targets from the same test program. This function
  is NOT thread-safe; only use this in single-threaded tests. */
  __attribute__((cold)) static void Reset() {
    // For the test-only function, we use the default memory order (sequential
    // consistency) because we might as well keep it simple, even though under
    // "only use in single-threaded tests" the memory order doesn't matter.
    function_.store(&LateBoundFunction::ChooseThenCall);
  }

  // Static globals must be trivially destructible.
  static_assert(std::is_trivially_destructible_v<std::atomic<FunctionPointer>>);

  // Note that the default value for this variable is provided out-of-line
  // below; it does NOT default to null.
  static std::atomic<FunctionPointer> function_;
};

template <typename ChooseFunctor>
std::atomic<decltype(ChooseFunctor()())>
    LateBoundFunction<ChooseFunctor>::function_ =
        &LateBoundFunction::ChooseThenCall;

}  // namespace
