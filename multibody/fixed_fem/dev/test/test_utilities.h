#pragma once

#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace fem {
namespace test {

/* Calculates an approximation of the condition number for the given matrix A.
 The (approximated) condition number is calculated via
     k(A) = σₘₐₓ / max(σₘᵢₙ,ε)
 where σₘₐₓ and σₘᵢₙ are the largest and smallest singular values (in magnitude)
 of A, respectively, and ε is machine epsilon.
 @tparam_nonsymbolic_scalar. */
template <typename T>
double CalcConditionNumber(const Eigen::Ref<const MatrixX<T>>& A);

}  // namespace test
}  // namespace fem
}  // namespace multibody
}  // namespace drake
