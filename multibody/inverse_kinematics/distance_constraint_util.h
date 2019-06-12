#pragma once

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
namespace internal {
void Distance(const MultibodyPlant<double>&, const systems::Context<double>&,
              const Frame<double>&, const Frame<double>&,
              const Eigen::Vector3d&, double distance, const Eigen::Vector3d&,
              const Eigen::Ref<const AutoDiffVecXd>&, double* distance_double);
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
