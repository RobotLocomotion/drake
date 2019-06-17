#pragma once

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
namespace internal {
/**
 * @param plant The plant for which the distance is computed.
 * @param context The context containing the generalized position for computing
 * the distance.
 * @param frameA The frame to which geometry A is attached.
 * @param frameB The frame to which geometry B is attached.
 * @param p_ACa The position of the witness point Ca measured and expressed in
 * frame A.
 * @param distance The signed distance between geometry A and B.
 * @param nhat_BA_W The unit length vector representing the gradient of the
 * signed distance field to geometry B, expressed in the world frame.
 * @param q The generalized position of the plant, it also stores the gradient
 * dq / dz, where z is some other variables.
 * @param[out] diatance_autodiff Containing the gradient of @p distance w.r.t
 * z (the same variable as shown up in the gradient of q).
 */
void CalcDistanceDerivatives(const MultibodyPlant<double>& plant,
                             const systems::Context<double>& context,
                             const Frame<double>& frameA,
                             const Frame<double>& frameB,
                             const Eigen::Vector3d& p_ACa, double distance,
                             const Eigen::Vector3d& nhat_BA_W,
                             const Eigen::Ref<const AutoDiffVecXd>& q,
                             AutoDiffXd* distance_autodiff);

/**
 * This is the overloaded version of Distance for the input q of double scalar
 * type, it just copies @p distance to @p distance_double.
 */
void CalcDistanceDerivatives(const MultibodyPlant<double>&,
                             const systems::Context<double>&,
                             const Frame<double>&, const Frame<double>&,
                             const Eigen::Vector3d&, double distance,
                             const Eigen::Vector3d&,
                             const Eigen::Ref<const AutoDiffVecXd>&,
                             double* distance_double);

/**
 * Check if the plant has registered its geometry with the SceneGraph.
 */
void CheckPlantIsConnectedToSceneGraph(
    const MultibodyPlant<double>& plant,
    const systems::Context<double>& plant_context);
}  // namespace internal
}  // namespace multibody
}  // namespace drake
