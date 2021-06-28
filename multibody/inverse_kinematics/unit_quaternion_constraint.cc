#include "drake/multibody/inverse_kinematics/unit_quaternion_constraint.h"

#include "drake/common/default_scalars.h"

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

template <typename T>
void AddUnitQuaternionConstraintOnPlant(
    const MultibodyPlant<T>& plant,
    const Eigen::Ref<const VectorX<symbolic::Variable>>& q_vars,
    solvers::MathematicalProgram* prog) {
  DRAKE_DEMAND(q_vars.rows() == plant.num_positions());
  // Loop through each body
  for (BodyIndex body_index{0}; body_index < plant.num_bodies(); ++body_index) {
    const auto& body = plant.get_body(body_index);
    if (body.has_quaternion_dofs()) {
      prog->AddConstraint(solvers::Binding<solvers::Constraint>(
          std::make_shared<UnitQuaternionConstraint>(),
          q_vars.segment<4>(body.floating_positions_start())));
    }
  }
}

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS((
    &AddUnitQuaternionConstraintOnPlant<T>
))

}  // namespace multibody
}  // namespace drake
