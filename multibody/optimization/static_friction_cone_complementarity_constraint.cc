#include "drake/multibody/optimization/static_friction_cone_complementarity_constraint.h"

#include <limits>

#include "drake/multibody/inverse_kinematics/kinematic_constraint_utilities.h"

namespace drake {
namespace multibody {

const double kInf = std::numeric_limits<double>::infinity();

namespace internal {

StaticFrictionConeComplementarityNonlinearConstraint::
    StaticFrictionConeComplementarityNonlinearConstraint(
        const ContactWrenchEvaluator* contact_wrench_evaluator,
        double complementarity_tolerance)
    : solvers::Constraint(
          4,
          contact_wrench_evaluator->plant().num_positions() +
              contact_wrench_evaluator->num_lambda() + 2,
          Eigen::Vector4d::Zero(),
          Eigen::Vector4d(kInf, 0, 0, complementarity_tolerance)),
      contact_wrench_evaluator_{contact_wrench_evaluator},
      alpha_var_{"alpha"},
      beta_var_{"beta"} {}

void StaticFrictionConeComplementarityNonlinearConstraint::
    UpdateComplementarityTolerance(double complementarity_tolerance) {
  Eigen::Vector4d upper_bound = this->upper_bound();
  upper_bound(3) = complementarity_tolerance;
  UpdateUpperBound(upper_bound);
}

solvers::Binding<internal::StaticFrictionConeComplementarityNonlinearConstraint>
StaticFrictionConeComplementarityNonlinearConstraint::MakeBinding(
    const ContactWrenchEvaluator* contact_wrench_evaluator,
    double complementarity_tolerance,
    const Eigen::Ref<const VectorX<symbolic::Variable>>& q_vars,
    const Eigen::Ref<const VectorX<symbolic::Variable>>& lambda_vars) {
  auto constraint =
      std::make_shared<StaticFrictionConeComplementarityNonlinearConstraint>(
          contact_wrench_evaluator, complementarity_tolerance);
  VectorX<symbolic::Variable> bound_vars(constraint->num_vars());
  bound_vars << q_vars, lambda_vars, constraint->alpha_var(),
      constraint->beta_var();
  return solvers::Binding<StaticFrictionConeComplementarityNonlinearConstraint>(
      constraint, bound_vars);
}

void StaticFrictionConeComplementarityNonlinearConstraint::DoEval(
    const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::VectorXd* y) const {
  AutoDiffVecXd y_autodiff(num_constraints());
  DoEval(x.cast<AutoDiffXd>(), &y_autodiff);
  *y = math::ExtractValue(y_autodiff);
}

void StaticFrictionConeComplementarityNonlinearConstraint::DoEval(
    const Eigen::Ref<const AutoDiffVecXd>& x, AutoDiffVecXd* y) const {
  y->resize(num_constraints());
  // Retrieve the variables.
  AutoDiffVecXd q, lambda;
  AutoDiffXd alpha, beta;
  DecomposeX(x, &q, &lambda, &alpha, &beta);
  // Update context with q.
  const auto& plant = contact_wrench_evaluator_->plant();
  systems::Context<AutoDiffXd>& context =
      // We use context to cache the kinematic results, so if q is different
      // from the position stored inside context, we will need to modify
      // context. But DoEval is a const method, hence it can only return the
      // const context. We use const_cast to remove the constness of the
      // context. If it could be guaranteed that the `q` was already set in the
      // context, this const_cast wouldn't be necessary.
      const_cast<systems::Context<AutoDiffXd>&>(
          contact_wrench_evaluator_->context());
  if (!internal::AreAutoDiffVecXdEqual(q, plant.GetPositions(context))) {
    plant.SetPositions(&context, q);
  }

  // Compute the contact wrench F_AB_W
  AutoDiffVecXd F_AB_W;
  contact_wrench_evaluator_->Eval(
      contact_wrench_evaluator_->ComposeVariableValues(context, lambda),
      &F_AB_W);
  Vector3<AutoDiffXd> f_AB_W = F_AB_W.tail<3>();
  // First get the signed distance result for the pair of geometries.
  // TODO(hongkai.dai): do not compute the signed distance results for every
  // pair of geometries, instead just compute the pair in the
  // contact_wrench_evaluator_
  const auto& query_port = plant.get_geometry_query_input_port();
  if (!query_port.HasValue(context)) {
    throw std::invalid_argument(
        "StaticEquilibriumConstraint: Cannot get a valid "
        "geometry::QueryObject. Please refer to AddMultibodyPlantSceneGraph "
        "on connecting MultibodyPlant to SceneGraph.");
  }
  const auto& query_object =
      query_port.Eval<geometry::QueryObject<AutoDiffXd>>(context);
  const geometry::SignedDistancePair<AutoDiffXd> signed_distance_pair =
      query_object.ComputeSignedDistancePairClosestPoints(
          contact_wrench_evaluator_->geometry_id_pair().first(),
          contact_wrench_evaluator_->geometry_id_pair().second());
  // Compute the friction.
  const geometry::ProximityProperties& geometryA_props =
      *query_object.inspector().GetProximityProperties(
          signed_distance_pair.id_A);
  const geometry::ProximityProperties& geometryB_props =
      *query_object.inspector().GetProximityProperties(
          signed_distance_pair.id_B);

  const CoulombFriction<double>& geometryA_friction =
      geometryA_props.GetProperty<CoulombFriction<double>>("material",
                                                           "coulomb_friction");
  const CoulombFriction<double>& geometryB_friction =
      geometryB_props.GetProperty<CoulombFriction<double>>("material",
                                                           "coulomb_friction");

  CoulombFriction<double> combined_friction =
      CalcContactFrictionFromSurfaceProperties(geometryA_friction,
                                               geometryB_friction);

  const auto& nhat_BA_W = signed_distance_pair.nhat_BA_W;
  // The constraint f_Wᵀ * ((μ² + 1)* n_W * n_Wᵀ - I) * f_W >= 0
  (*y)(0) = f_AB_W.dot(((std::pow(combined_friction.static_friction(), 2) + 1) *
                            nhat_BA_W * nhat_BA_W.transpose() -
                        Eigen::Matrix3d::Identity()) *
                       f_AB_W);

  // The constraint n̂_AB_W.dot(f_AB_W) - α = 0
  (*y)(1) = -nhat_BA_W.dot(f_AB_W) - alpha;

  // The constraint sdf - β = 0
  (*y)(2) = signed_distance_pair.distance - beta;

  // The constraint α * β <= ε
  (*y)(3) = alpha * beta;
}

void StaticFrictionConeComplementarityNonlinearConstraint::DoEval(
    const Eigen::Ref<const VectorX<symbolic::Variable>>&,
    VectorX<symbolic::Expression>*) const {
  throw std::runtime_error(
      "StaticFrictionConeComplementarityNonlinearConstraint: does not support "
      "Eval with symbolic variable and expressions.");
}
}  // namespace internal

solvers::Binding<internal::StaticFrictionConeComplementarityNonlinearConstraint>
AddStaticFrictionConeComplementarityConstraint(
    const ContactWrenchEvaluator* contact_wrench_evaluator,
    double complementarity_tolerance,
    const Eigen::Ref<const VectorX<symbolic::Variable>>& q_vars,
    const Eigen::Ref<const VectorX<symbolic::Variable>>& lambda_vars,
    solvers::MathematicalProgram* prog) {
  // Create the nonlinear binding.
  auto nonlinear_binding =
      internal::StaticFrictionConeComplementarityNonlinearConstraint::
          MakeBinding(contact_wrench_evaluator, complementarity_tolerance,
                      q_vars, lambda_vars);
  // Add the variable α,β to the program.
  const Vector2<symbolic::Variable> alpha_beta_var(
      nonlinear_binding.evaluator()->alpha_var(),
      nonlinear_binding.evaluator()->beta_var());
  prog->AddDecisionVariables(alpha_beta_var);
  // Add the nonlinear binding to the program.
  prog->AddConstraint(nonlinear_binding);
  // Add constraint that α >= 0, β >= 0
  prog->AddBoundingBoxConstraint(
      Eigen::Vector2d::Zero(), Eigen::Vector2d::Constant(kInf), alpha_beta_var);
  return nonlinear_binding;
}
}  // namespace multibody
}  // namespace drake
