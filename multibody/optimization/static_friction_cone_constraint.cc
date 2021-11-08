#include "drake/multibody/optimization/static_friction_cone_constraint.h"

#include <limits>
#include <vector>

#include "drake/multibody/inverse_kinematics/kinematic_constraint_utilities.h"

namespace drake {
namespace multibody {
const double kInf = std::numeric_limits<double>::infinity();

StaticFrictionConeConstraint::StaticFrictionConeConstraint(
    const ContactWrenchEvaluator* contact_wrench_evaluator)
    : solvers::Constraint(2 /* number of constraints */,
                          contact_wrench_evaluator->plant().num_positions() +
                              contact_wrench_evaluator->num_lambda(),
                          Eigen::Vector2d::Zero(),
                          Eigen::Vector2d::Constant(kInf)),
      contact_wrench_evaluator_{contact_wrench_evaluator} {}

void StaticFrictionConeConstraint::DoEval(
    const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::VectorXd* y) const {
  AutoDiffVecXd y_autodiff(num_constraints());
  DoEval(x.cast<AutoDiffXd>(), &y_autodiff);
  *y = math::ExtractValue(y_autodiff);
}

void StaticFrictionConeConstraint::DoEval(
    const Eigen::Ref<const AutoDiffVecXd>& x, AutoDiffVecXd* y) const {
  y->resize(num_constraints());
  AutoDiffVecXd q, lambda;
  DecomposeX(x, &q, &lambda);
  // Update context with q.
  const auto& plant = contact_wrench_evaluator_->plant();
  systems::Context<AutoDiffXd>& context =
      const_cast<systems::Context<AutoDiffXd>&>(
          contact_wrench_evaluator_->context());
  if (!internal::AreAutoDiffVecXdEqual(q, plant.GetPositions(context))) {
    plant.SetPositions(&context, q);
  }
  // Compute the contact wrench F_Cb_W = [τ; f] where we only enforce
  // constraints on f
  AutoDiffVecXd F_Cb_W;
  contact_wrench_evaluator_->Eval(
      contact_wrench_evaluator_->ComposeVariableValues(context, lambda),
      &F_Cb_W);
  // Only the contact force f_Cb_W is constrained.
  Vector3<AutoDiffXd> f_Cb_W = F_Cb_W.tail<3>();

  // First get the signed distance result for the pair of geometries.
  // TODO(hongkai.dai): do not compute the signed distance results for every
  // pair of geometries, instead just compute the pair in the
  // contact_wrench_evaluator_
  const auto& query_port = plant.get_geometry_query_input_port();
  if (!query_port.HasValue(context)) {
    throw std::invalid_argument(
        "StaticFrictionConeConstraint: Cannot get a valid "
        "geometry::QueryObject. Please refer to AddMultibodyPlantSceneGraph "
        "on connecting MultibodyPlant to SceneGraph.");
  }
  const auto& query_object =
      query_port.Eval<geometry::QueryObject<AutoDiffXd>>(context);
  const std::vector<geometry::SignedDistancePair<AutoDiffXd>>
      signed_distance_pairs =
          query_object.ComputeSignedDistancePairwiseClosestPoints();
  bool found_geometry_pair = false;
  for (const auto& signed_distance_pair : signed_distance_pairs) {
    // In SceneGraph::ComputeSignedDistancePairwiseClosestPoints(), the geometry
    // IDs in each signed_distance_pair is guaranteed to be sorted, in the same
    // order as the SortedPair returned by geometry_id_pair().
    if (signed_distance_pair.id_A ==
            contact_wrench_evaluator_->geometry_id_pair().first() &&
        signed_distance_pair.id_B ==
            contact_wrench_evaluator_->geometry_id_pair().second()) {
      found_geometry_pair = true;
      // Compute the friction.
      const geometry::ProximityProperties& geometryA_props =
          *query_object.inspector().GetProximityProperties(
              signed_distance_pair.id_A);
      const geometry::ProximityProperties& geometryB_props =
          *query_object.inspector().GetProximityProperties(
              signed_distance_pair.id_B);

      const CoulombFriction<double>& geometryA_friction =
          geometryA_props.GetProperty<CoulombFriction<double>>(
              "material", "coulomb_friction");
      const CoulombFriction<double>& geometryB_friction =
          geometryB_props.GetProperty<CoulombFriction<double>>(
              "material", "coulomb_friction");

      CoulombFriction<double> combined_friction =
          CalcContactFrictionFromSurfaceProperties(geometryA_friction,
                                                   geometryB_friction);

      const auto& nhat_BA_W = signed_distance_pair.nhat_BA_W;
      // The constraint n̂_AB_W.dot(f_Cb_W) >= 0
      (*y)(0) = -nhat_BA_W.dot(f_Cb_W);

      // The constraint f_Cb_Wᵀ * ((μ² + 1)* n̂_AB_W * n̂_AB_Wᵀ - I) * f_Cb_W >= 0
      (*y)(1) =
          f_Cb_W.dot(((std::pow(combined_friction.static_friction(), 2) + 1) *
                          nhat_BA_W * nhat_BA_W.transpose() -
                      Eigen::Matrix3d::Identity()) *
                     f_Cb_W);
      break;
    }
  }
  if (!found_geometry_pair) {
    throw std::runtime_error(
        "StaticFrictionConeConstraint: the input contact_wrench_evaluator "
        "contains a pair of geometries that have not been registered to the "
        "SceneGraph for distance computation.");
  }
}

void StaticFrictionConeConstraint::DoEval(
    const Eigen::Ref<const VectorX<symbolic::Variable>>&,
    VectorX<symbolic::Expression>*) const {
  throw std::runtime_error(
      "StaticFrictionConeConstraint: does not support Eval with symbolic "
      "variables and expressions.");
}

}  // namespace multibody
}  // namespace drake
