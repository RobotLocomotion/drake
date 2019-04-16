#include "drake/multibody/optimization/static_friction_cone_complementary_constraint.h"

#include <limits>

#include "drake/multibody/inverse_kinematics/kinematic_constraint_utilities.h"

namespace drake {
namespace multibody {
namespace internal {
const double kInf = std::numeric_limits<double>::infinity();
StaticFrictionConeComplementaryNonlinearConstraint::
    StaticFrictionConeComplementaryNonlinearConstraint(
        const ContactWrenchEvaluator* contact_wrench_evaluator,
        double complementary_tolerance)
    : solvers::Constraint(4,
                          contact_wrench_evaluator->plant().num_positions() +
                              contact_wrench_evaluator->num_lambda() + 2,
                          Eigen::Vector4d::Zero(),
                          Eigen::Vector4d(kInf, 0, 0, complementary_tolerance)),
      contact_wrench_evaluator_{contact_wrench_evaluator},
      alpha_var_{"alpha"},
      beta_var_{"beta"} {}

void StaticFrictionConeComplementaryNonlinearConstraint::
    UpdateComplementaryTolerance(double complementary_tolerance) {
  Eigen::Vector4d upper_bound = this->upper_bound();
  upper_bound(3) = complementary_tolerance;
  UpdateUpperBound(upper_bound);
}

void StaticFrictionConeComplementaryNonlinearConstraint::DoEval(
    const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::VectorXd* y) const {
  AutoDiffVecXd y_autodiff(num_constraints());
  DoEval(x.cast<AutoDiffXd>(), &y_autodiff);
  *y = math::autoDiffToValueMatrix(y_autodiff);
}

void StaticFrictionConeComplementaryNonlinearConstraint::DoEval(
    const Eigen::Ref<const AutoDiffVecXd>& x, AutoDiffVecXd* y) const {
  // Retrieve the variables.
  AutoDiffVecXd q, lambda;
  AutoDiffXd alpha, beta;
  DecomposeX(x, &q, &lambda, &alpha, &beta);
  // Update context with q.
  const auto& plant = contact_wrench_evaluator_->plant();
  systems::Context<AutoDiffXd>* context =
      contact_wrench_evaluator_->get_mutable_context();
  if (!internal::AreAutoDiffVecXdEqual(q, plant.GetPositions(*context))) {
    plant.SetPositions(context, q);
  }

  // Compute the contact wrench F_AB_W
  AutoDiffVecXd F_AB_W;
  contact_wrench_evaluator_->Eval(
      contact_wrench_evaluator_->ComposeVariableValues(*context, lambda),
      &F_AB_W);
  Vector3<AutoDiffXd> f_AB_W = F_AB_W.tail<3>();
  // First get the signed distance result for the pair of geometries.
  // TODO(hongkai.dai): do not compute the signed distance results for every
  // pair of geometries, instead just compute the pair in the
  // contact_wrench_evaluator_
  const auto& query_port = plant.get_geometry_query_input_port();
  if (!query_port.HasValue(*context)) {
    throw std::invalid_argument(
        "StaticEquilibriumConstraint: Cannot get a valid "
        "geometry::QueryObject. Please refer to AddMultibodyPlantSceneGraph "
        "on connecting MultibodyPlant to SceneGraph.");
  }
  const auto& query_object =
      query_port.Eval<geometry::QueryObject<AutoDiffXd>>(*context);
  const std::vector<geometry::SignedDistancePair<double>>
      signed_distance_pairs =
          query_object.ComputeSignedDistancePairwiseClosestPoints();
  //const geometry::SceneGraphInspector<AutoDiffXd>& inspector =
  //    query_object.inspector();
  bool found_geometry_pair = false;
  for (const auto& signed_distance_pair : signed_distance_pairs) {
    if (signed_distance_pair.id_A ==
            contact_wrench_evaluator_->geometry_id_pair().first &&
        signed_distance_pair.id_B ==
            contact_wrench_evaluator_->geometry_id_pair().second) {
      found_geometry_pair = true;
      // Compute the friction.
      const CoulombFriction<double>& geometryA_friction =
          plant.default_coulomb_friction(signed_distance_pair.id_A);
      const CoulombFriction<double>& geometryB_friction =
          plant.default_coulomb_friction(signed_distance_pair.id_B);
      CoulombFriction<double> combined_friction =
          CalcContactFrictionFromSurfaceProperties(geometryA_friction,
                                                   geometryB_friction);

      const auto& nhat_BA_W = signed_distance_pair.nhat_BA_W;
      // The constraint f_Wᵀ * ((μ² + 1)* n_W * n_Wᵀ - I) * f_W >= 0
      (*y)(0) =
          f_AB_W.dot(((std::pow(combined_friction.static_friction(), 2) + 1) *
                          nhat_BA_W * nhat_BA_W.transpose() -
                      Eigen::Matrix3d::Identity()) *
                     f_AB_W);

      // The constraint n̂_AB_W.dot(f_AB_W) - α = 0
      (*y)(1) = -nhat_BA_W.dot(f_AB_W) - alpha;

      // The constraint sdf - β = 0
      (*y)(2) = signed_distance_pair.distance - beta;

      // The constraint α * β <= ε
      (*y)(3) = alpha * beta;

      break;
    }
  }
  if (!found_geometry_pair) {
    throw std::runtime_error(
        "StaticFrictionConeComplementaryNonlinearConstraint: the input "
        "contact_wrench_evaluator contains a pair of geometry, that has not "
        "been registered to the SceneGraph for distance computation.");
  }
}

void StaticFrictionConeComplementaryNonlinearConstraint::DoEval(
    const Eigen::Ref<const VectorX<symbolic::Variable>>&,
    VectorX<symbolic::Expression>*) const {
  throw std::runtime_error(
      "StaticEquilibriumConstraint: does not support Eval with symbolic "
      "variable and expressions.");
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
