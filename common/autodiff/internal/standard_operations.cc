/* clang-format off to disable clang-format-includes */
// NOLINTNEXTLINE(build/include) ...
#include "drake/common/autodiff/auto_diff.h"
/* clang-format on */

#include <ostream>

namespace drake {
namespace autodiff {

std::ostream& operator<<(std::ostream& s, const AutoDiff& x) {
  return s << x.value();
}

}  // namespace autodiff
}  // namespace drake
