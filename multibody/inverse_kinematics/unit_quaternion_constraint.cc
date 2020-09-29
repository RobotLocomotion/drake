#include "drake/multibody/inverse_kinematics/unit_quaternion_constraint.h"

namespace drake {
namespace multibody {
UnitQuaternionConstraint::UnitQuaternionConstraint()
    : solvers::Constraint(1, 4, Vector1d(1), Vector1d(1)) {}

void UnitQuaternionConstraint::DoEval(
    const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::VectorXd* y) const {
  DoEvalGeneric(x, y);
}

void UnitQuaternionConstraint::DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
                                      AutoDiffVecXd* y) const {
  DoEvalGeneric(x, y);
}

void UnitQuaternionConstraint::DoEval(
    const Eigen::Ref<const VectorX<symbolic::Variable>>& x,
    VectorX<symbolic::Expression>* y) const {
  DoEvalGeneric(x, y);
}

}  // namespace multibody
}  // namespace drake
