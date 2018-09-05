#pragma once

#include <limits>

#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/multibody_tree/multibody_tree_context.h"

namespace drake {
namespace multibody {
namespace internal {
/**
 * Determines if a and b are equal. a equals to b if they have the same value
 * and gradients.
 * TODO(hongkai.dai) implement and use std::equal_to<> for comparing Eigen
 * vector of AutoDiffXd.
 */
bool AreAutoDiffVecXdEqual(const Eigen::Ref<const AutoDiffVecXd>& a,
                           const Eigen::Ref<const AutoDiffVecXd>& b);

/**
 * Check if the generalized positions in @p mbt_context are the same as @p q.
 * If they are not the same, then reset @p mbt_context's generalized positions
 * to q. Otherwise, leave @p mbt_context unchanged.
 * The intention is to avoid dirtying the computation cache, given it is
 * ticket-based rather than hash-based.
 */
void UpdateContextConfiguration(const Eigen::Ref<const VectorX<AutoDiffXd>>& q,
                                MultibodyTreeContext<AutoDiffXd>* mbt_context);

/**
 * Normalize an Eigen vector of doubles. Throw a logic error if the vector is
 * close to zero. This function is used in the constructor of some kinematic
 * constraints.
 */
template <typename DerivedA>
typename std::enable_if<
    is_eigen_vector_of<DerivedA, double>::value,
    Eigen::Matrix<double, DerivedA::RowsAtCompileTime, 1>>::type
NormalizeVector(const Eigen::MatrixBase<DerivedA>& a) {
  const double a_norm = a.norm();
  if (a_norm < 100 * a.rows() * std::numeric_limits<double>::epsilon()) {
    throw std::invalid_argument("a is close to a zero vector.");
  }
  return a / a_norm;
}
}  // namespace internal
}  // namespace multibody
}  // namespace drake
