// NOLINTNEXTLINE(build/include): Its header file is included in symbolic.h.
#include "drake/common/drake_assert.h"
#include "drake/common/symbolic.h"

namespace drake {
namespace symbolic {
Eigen::Matrix<Monomial, Eigen::Dynamic, 1> MonomialBasis(const Variables& vars,
                                                         const int degree) {
  return internal::ComputeMonomialBasis<Eigen::Dynamic>(vars, degree);
}
}  // namespace symbolic
}  // namespace drake
