#pragma once

#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace fem {
namespace test {

/* Calculates the condition number for the given matrix A.
 The condition number is calculated via
     k(A) = σₘₐₓ / σₘᵢₙ
 where σₘₐₓ and σₘᵢₙ are the largest and smallest singular values (in magnitude)
 of A.
 @pre A is invertible.
 @tparam_nonsymbolic_scalar. */
template <typename T>
double CalcConditionNumber(const Eigen::Ref<const MatrixX<T>>& A);

}  // namespace test
}  // namespace fem
}  // namespace multibody
}  // namespace drake
