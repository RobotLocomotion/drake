#pragma once

#include <limits>

#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/context.h"

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
 * Check if the generalized positions in @p context are the same as @p q.
 * If they are not the same, then reset @p context's generalized positions
 * to @p q. Otherwise, leave @p context unchanged.
 * The intention is to avoid dirtying the computation cache, given it is
 * ticket-based rather than hash-based.
 */
void UpdateContextConfiguration(drake::systems::Context<double>* context,
                                const MultibodyPlant<double>& plant,
                                const Eigen::Ref<const VectorX<double>>& q);

void UpdateContextConfiguration(drake::systems::Context<double>* context,
                                const MultibodyPlant<double>& plant,
                                const Eigen::Ref<const VectorX<AutoDiffXd>>& q);

void UpdateContextConfiguration(systems::Context<AutoDiffXd>* context,
                                const MultibodyPlant<AutoDiffXd>& plant,
                                const Eigen::Ref<const AutoDiffVecXd>& q);

/**
 * Normalize an Eigen vector of doubles. This function is used in the
 * constructor of some kinematic constraints.
 * @throws std::invalid_argument if the vector is close to zero.
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

/**
 * If `plant` is not nullptr, return a reference to the MultibodyPlant to which
 * it points.
 * @throws std::invalid_argument if `plant` is nullptr.
 */
template <typename T>
const MultibodyPlant<T>& RefFromPtrOrThrow(
    const MultibodyPlant<T>* const plant) {
  if (plant == nullptr) throw std::invalid_argument("plant is nullptr.");
  return *plant;
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
