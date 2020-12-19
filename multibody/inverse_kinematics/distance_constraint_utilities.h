#pragma once

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
namespace internal {
/*
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
 * @param[out] distance_autodiff Containing the gradient of @p distance w.r.t
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

/*
 * This is the overloaded version of CalcDistanceDerivatives for plant being
 * MultibodyPlant<AutoDiffXd> instead of MultibodyPlant<double>.
 */
void CalcDistanceDerivatives(const MultibodyPlant<AutoDiffXd>& plant,
                             const systems::Context<AutoDiffXd>& context,
                             const Frame<AutoDiffXd>& frameA,
                             const Frame<AutoDiffXd>& frameB,
                             const Vector3<AutoDiffXd>& p_ACa,
                             const AutoDiffXd& distance_autodiff,
                             const Vector3<AutoDiffXd>& nhat_BA_W,
                             const Eigen::Ref<const Eigen::VectorXd>& q,
                             double* distance);

/*
 * This is the overloaded version of CalcDistanceDerivatives for the input q of
 * double scalar type, it just copies @p distance_in to @p distance_out.
 */
template <typename T>
void CalcDistanceDerivatives(const MultibodyPlant<T>&,
                             const systems::Context<T>&, const Frame<T>&,
                             const Frame<T>&, const Vector3<T>&, T distance_in,
                             const Vector3<T>&,
                             const Eigen::Ref<const VectorX<T>>&,
                             T* distance_out) {
  *distance_out = distance_in;
}

/*
 * Check if the plant has registered its geometry with the SceneGraph.
 */
template <typename T>
void CheckPlantIsConnectedToSceneGraph(
    const MultibodyPlant<T>& plant, const systems::Context<T>& plant_context) {
  if (!plant.geometry_source_is_registered()) {
    throw std::invalid_argument(
        "Kinematic constraint: MultibodyPlant has not registered "
        "with a SceneGraph yet. Please refer to "
        "AddMultibodyPlantSceneGraph on how to connect MultibodyPlant to "
        "SceneGraph.");
  }
  const auto& query_port = plant.get_geometry_query_input_port();
  if (!query_port.HasValue(plant_context)) {
    throw std::invalid_argument(
        "Kinematic constraint: Cannot get a valid "
        "geometry::QueryObject. Either the plant's geometry query input port "
        "is not properly connected to the SceneGraph's geometry query output "
        "port, or the plant_context_ is incorrect. Please refer to "
        "AddMultibodyPlantSceneGraph on connecting MultibodyPlant to "
        "SceneGraph.");
  }
}
}  // namespace internal
}  // namespace multibody
}  // namespace drake
