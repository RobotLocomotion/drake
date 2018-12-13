#include "drake/multibody/inverse_kinematics/kinematic_constraint_utilities.h"

namespace drake {
namespace multibody {
namespace internal {
bool AreAutoDiffVecXdEqual(const Eigen::Ref<const VectorX<AutoDiffXd>>& a,
                           const Eigen::Ref<const VectorX<AutoDiffXd>>& b) {
  if (a.rows() != b.rows()) {
    return false;
  }
  if (math::autoDiffToValueMatrix(a) != math::autoDiffToValueMatrix(b)) {
    return false;
  }
  const Eigen::MatrixXd a_gradient = math::autoDiffToGradientMatrix(a);
  const Eigen::MatrixXd b_gradient = math::autoDiffToGradientMatrix(b);
  if (a_gradient.rows() != b_gradient.rows() ||
      a_gradient.cols() != b_gradient.cols()) {
    return false;
  }
  return a_gradient == b_gradient;
}

void UpdateContextConfiguration(const Eigen::Ref<const VectorX<AutoDiffXd>>& q,
                                MultibodyTreeContext<AutoDiffXd>* mbt_context) {
  if (!AreAutoDiffVecXdEqual(q, mbt_context->get_positions())) {
    mbt_context->get_mutable_positions() = q;
  }
}

void UpdateContextConfiguration(
    drake::systems::Context<double>* context,
    const drake::multibody::multibody_plant::MultibodyPlant<double>& plant,
    const Eigen::Ref<const VectorX<double>>& q) {
  DRAKE_ASSERT(context);
  if (q != plant.GetPositions(*context)) {
    plant.SetPositions(context, q);
  }
}

const multibody_plant::MultibodyPlant<double>& RefFromPtrOrThrow(
    const multibody_plant::MultibodyPlant<double>* const plant) {
  DRAKE_THROW_UNLESS(plant != nullptr);
  return *plant;
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
