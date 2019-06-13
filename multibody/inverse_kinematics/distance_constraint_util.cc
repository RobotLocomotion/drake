#include "drake/multibody/inverse_kinematics/distance_constraint_util.h"

#include "drake/math/autodiff_gradient.h"

namespace drake {
namespace multibody {
namespace internal {
void Distance(const MultibodyPlant<double>&, const systems::Context<double>&,
              const Frame<double>&, const Frame<double>&,
              const Eigen::Vector3d&, double distance, const Eigen::Vector3d&,
              const Eigen::Ref<const AutoDiffVecXd>&, double* distance_double) {
  *distance_double = distance;
}

void Distance(const MultibodyPlant<double>& plant,
              const systems::Context<double>& context,
              const Frame<double>& frameA, const Frame<double>& frameB,
              const Eigen::Vector3d& p_ACa, double distance,
              const Eigen::Vector3d& nhat_BA_W,
              const Eigen::Ref<const AutoDiffVecXd>& q,
              AutoDiffXd* distance_autodiff) {
  // The distance is d = sign * |p_CbCa_B|, where the
  // closest points are Ca on object A, and Cb on object B.
  // So the gradient ∂d/∂q = p_CbCa_W * ∂p_BCa_B/∂q / d (Note that
  // ∂p_BCa_B/∂q = ∂p_CbCa_B/∂q).
  //
  // Since dividing by d is undefined when d = 0, and p_CbCa_W / d =
  // nhat_BA_W whenever d ≠ 0, we use ∂d/∂q = nhat_BA_W′ * ∂p_BCa_B/∂q. This
  // allows us to compute a gradient at d = 0 in certain cases (See
  // geometry::SignedDistancePair for details).
  Eigen::Matrix<double, 3, Eigen::Dynamic> Jq_v_BCa_W(3, plant.num_positions());
  plant.CalcJacobianTranslationalVelocity(context, JacobianWrtVariable::kQDot,
                                          frameA, p_ACa, frameB,
                                          plant.world_frame(), &Jq_v_BCa_W);
  const Eigen::RowVectorXd ddistance_dq = nhat_BA_W.transpose() * Jq_v_BCa_W;
  distance_autodiff->value() = distance;
  distance_autodiff->derivatives() =
      ddistance_dq * math::autoDiffToGradientMatrix(q);
}

void CheckPlantConnectSceneGraph(
    const MultibodyPlant<double>& plant,
    const systems::Context<double>& plant_context) {
  if (!plant.geometry_source_is_registered()) {
    throw std::invalid_argument(
        "CheckPlantConnectSceneGraph(): MultibodyPlant has not registered "
        "with a SceneGraph yet. Please refer to "
        "AddMultibodyPlantSceneGraph on how to connect MultibodyPlant to "
        "SceneGraph.");
  }
  const auto& query_port = plant.get_geometry_query_input_port();
  if (!query_port.HasValue(plant_context)) {
    throw std::invalid_argument(
        "CheckPlantConnectSceneGraph(): Cannot get a valid "
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
