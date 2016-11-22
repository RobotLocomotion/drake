#include "drake/common/double_overloads.h"

namespace drake {
double if_then_else(const bool f_cond, const double v_then,
                    const double v_else) {
  return f_cond ? v_then : v_else;
}
}  // namespace drake
