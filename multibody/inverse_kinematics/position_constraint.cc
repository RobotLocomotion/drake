#include "drake/multibody/inverse_kinematics/position_constraint.h"

#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/inverse_kinematics/kinematic_constraint_utilities.h"

using drake::multibody::internal::RefFromPtrOrThrow;
using drake::multibody::internal::UpdateContextConfiguration;

namespace drake {
namespace multibody {
PositionConstraint::PositionConstraint(
    const multibody_plant::MultibodyPlant<double>* const plant,
    const Frame<double>& frameB, const Eigen::Ref<const Eigen::Vector3d>& p_BQ,
    const Frame<double>& frameA,
    const Eigen::Ref<const Eigen::Vector3d>& p_AQ_lower,
    const Eigen::Ref<const Eigen::Vector3d>& p_AQ_upper,
    systems::Context<double>* context)
    : solvers::Constraint(3, RefFromPtrOrThrow(plant).num_positions(),
                          p_AQ_lower, p_AQ_upper),
      plant_(RefFromPtrOrThrow(plant)),
      frameB_{frameB},
      frameA_{frameA},
      p_BQ_{p_BQ},
      context_{context} {
  frameB_.HasThisParentTreeOrThrow(&plant_.tree());
  frameA_.HasThisParentTreeOrThrow(&plant_.tree());
  DRAKE_DEMAND(context);
}

void PositionConstraint::DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
                                Eigen::VectorXd* y) const {
  // TODO(avalenzu): Re-work to avoid round-trip through AutoDiffXd (#10205).
  AutoDiffVecXd y_t;
  Eval(math::initializeAutoDiff(x), &y_t);
  *y = math::autoDiffToValueMatrix(y_t);
}

void PositionConstraint::DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
                                AutoDiffVecXd* y) const {
  y->resize(3);
  UpdateContextConfiguration(context_, plant_, math::autoDiffToValueMatrix(x));
  Eigen::Vector3d p_AQ{};
  plant_.tree().CalcPointsPositions(*context_, frameB_, p_BQ_, frameA_, &p_AQ);
  Eigen::MatrixXd Jq_V_ABq(6, plant_.num_positions());
  plant_.tree().CalcJacobianSpatialVelocity(*context_,
                                            JacobianWrtVariable::kQDot, frameB_,
                                            p_BQ_, frameA_, frameA_, &Jq_V_ABq);
  *y = math::initializeAutoDiffGivenGradientMatrix(
      p_AQ, Jq_V_ABq.bottomRows<3>() * math::autoDiffToGradientMatrix(x));
}

}  // namespace multibody
}  // namespace drake
