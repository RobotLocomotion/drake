#include "drake/common/symbolic_monomial_util.h"

#include "drake/common/drake_assert.h"

namespace drake {
namespace symbolic {
Eigen::Matrix<Monomial, Eigen::Dynamic, 1> MonomialBasis(const Variables& vars,
                                                         const int degree) {
  return internal::ComputeMonomialBasis<Eigen::Dynamic>(vars, degree);
}
}  // namespace symbolic
}  // namespace drake
