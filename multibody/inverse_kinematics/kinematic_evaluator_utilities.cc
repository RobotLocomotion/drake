#include "drake/multibody/inverse_kinematics/kinematic_evaluator_utilities.h"

namespace drake {
namespace multibody {
namespace internal {

void UpdateContextConfiguration(drake::systems::Context<double>* context,
                                const MultibodyPlant<double>& plant,
                                const Eigen::Ref<const VectorX<double>>& q) {
  DRAKE_ASSERT(context != nullptr);
  if (q != plant.GetPositions(*context)) {
    plant.SetPositions(context, q);
  }
}

void UpdateContextConfiguration(drake::systems::Context<double>* context,
                                const MultibodyPlant<double>& plant,
                                const Eigen::Ref<const AutoDiffVecXd>& q) {
  return UpdateContextConfiguration(context, plant, math::ExtractValue(q));
}

void UpdateContextConfiguration(systems::Context<AutoDiffXd>* context,
                                const MultibodyPlant<AutoDiffXd>& plant,
                                const Eigen::Ref<const AutoDiffVecXd>& q) {
  DRAKE_ASSERT(context != nullptr);
  if (!math::AreAutoDiffVecXdEqual(q, plant.GetPositions(*context))) {
    plant.SetPositions(context, q);
  }
}

void UpdateContextPositionsAndVelocities(
    systems::Context<double>* context, const MultibodyPlant<double>& plant,
    const Eigen::Ref<const Eigen::VectorXd>& q_v) {
  DRAKE_ASSERT(context != nullptr);
  if (q_v != plant.GetPositionsAndVelocities(*context)) {
    plant.SetPositionsAndVelocities(context, q_v);
  }
}

void UpdateContextPositionsAndVelocities(
    systems::Context<double>* context, const MultibodyPlant<double>& plant,
    const Eigen::Ref<const AutoDiffVecXd>& q_v) {
  return UpdateContextPositionsAndVelocities(context, plant,
                                             math::ExtractValue(q_v));
}

void UpdateContextPositionsAndVelocities(
    systems::Context<AutoDiffXd>* context,
    const MultibodyPlant<AutoDiffXd>& plant,
    const Eigen::Ref<const AutoDiffVecXd>& q_v) {
  DRAKE_ASSERT(context != nullptr);
  if (!math::AreAutoDiffVecXdEqual(q_v,
                                   plant.GetPositionsAndVelocities(*context))) {
    plant.SetPositionsAndVelocities(context, q_v);
  }
}
}  // namespace internal
}  // namespace multibody
}  // namespace drake
