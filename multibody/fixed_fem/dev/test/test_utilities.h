#pragma once

#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace fem {
namespace test {

/* Calculates an approximation of the condition number for the given matrix A.
 The condition number is calculated via k(A) = |λₘₐₓ|/|λₘᵢₙ| where the λ's are
 the singular values of A sorted in according to their absolute values.
 The condition number is only an approximation when the smallest eigen value is
 smaller than machine epsilon, in which case the ratio of |λₘₐₓ| and the machine
 epsilon is returned.
 @tparam_nonsymbolic_scalar. */
template <typename T>
double CalcConditionNumber(const Eigen::Ref<const MatrixX<T>>& A);

}  // namespace test
}  // namespace fem
}  // namespace multibody
}  // namespace drake
