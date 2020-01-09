#pragma once

/// @file
/// A benchmarking helper.

#include <chrono>
#include <utility>

namespace drake {
namespace common {
namespace test {

/// Returns the elapsed time of `func(args)`, in seconds.
template <typename F, typename... Args>
[[nodiscard]] static double MeasureExecutionTime(F func, Args&&... args) {
  using clock = std::chrono::steady_clock;

  const clock::time_point start = clock::now();
  func(std::forward<Args>(args)...);
  const clock::time_point end = clock::now();

  return std::chrono::duration<double>(end - start).count();
}

}  // namespace test
}  // namespace common
}  // namespace drake
