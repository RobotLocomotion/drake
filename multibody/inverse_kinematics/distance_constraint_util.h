#pragma once

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
namespace internal {
/**
 * This is the overloaded function used in the distance-related constraint.
 * for the double version, it just copies @p distance to @p distance_double.
 * for the autodiff version, it needs to compute the gradient of the distance
 * w.r.t q, given the witness points and the distance of the two geometries.
 */
void Distance(const MultibodyPlant<double>&, const systems::Context<double>&,
              const Frame<double>&, const Frame<double>&,
              const Eigen::Vector3d&, double distance, const Eigen::Vector3d&,
              const Eigen::Ref<const AutoDiffVecXd>&, double* distance_double);
/**
 * @param frameA The frame to which geometry A is attached.
 * @param frameB The frame to which geometry B is attached.
 * @param p_ACa The position of the witness point Ca measured and expressed in
 * frame A.
 * @param p_BCb The position of the witness point Cb measured and expressed in
 * frame B.
 * @param nhat_BA_W The unit length vector representing the gradient of the
 * distance field to geometry A, expressed in the world frame.
 * @param q The generalized position of the plant.
 */
void Distance(const MultibodyPlant<double>& plant,
              const systems::Context<double>& context,
              const Frame<double>& frameA, const Frame<double>& frameB,
              const Eigen::Vector3d& p_ACa, double distance,
              const Eigen::Vector3d& nhat_BA_W,
              const Eigen::Ref<const AutoDiffVecXd>& q,
              AutoDiffXd* distance_autodiff);
}  // namespace internal
}  // namespace multibody
}  // namespace drake
