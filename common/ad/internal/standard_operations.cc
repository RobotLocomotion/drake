// Our header file doesn't stand on its own; it should only ever be included
// indirectly via auto_diff.h

/* clang-format off to disable clang-format-includes */
// NOLINTNEXTLINE(build/include)
#include "drake/common/ad/auto_diff.h"
/* clang-format on */

#include <ostream>

#include <fmt/format.h>

namespace drake {
namespace ad {

AutoDiff ceil(AutoDiff x) {
  x.value() = std::ceil(x.value());
  x.partials().SetZero();
  return x;
}

AutoDiff floor(AutoDiff x) {
  x.value() = std::floor(x.value());
  x.partials().SetZero();
  return x;
}

AutoDiff round(AutoDiff x) {
  x.value() = std::round(x.value());
  x.partials().SetZero();
  return x;
}

AutoDiff nexttoward(AutoDiff from, long double to) {
  from.value() = std::nexttoward(from.value(), to);
  from.partials().SetZero();
  return from;
}

std::ostream& operator<<(std::ostream& s, const AutoDiff& x) {
  return s << fmt::format("{}", x.value());
}

}  // namespace ad
}  // namespace drake
