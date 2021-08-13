#include "drake/multibody/optimization/sliding_friction_complementarity_constraint.h"

#include <cmath>
#include <limits>
#include <memory>

#include "drake/multibody/inverse_kinematics/kinematic_constraint_utilities.h"

namespace drake {

using std::pow;

namespace multibody {
const double kInf = std::numeric_limits<double>::infinity();
namespace internal {
namespace {
Eigen::Matrix<double, 11, 1> GetConstraintLowerBound(
    double complementarity_tolerance) {
  Eigen::Matrix<double, 11, 1> lower_bound =
      Eigen::Matrix<double, 11, 1>::Zero();
  lower_bound(3) = -complementarity_tolerance;
  lower_bound(4) = -complementarity_tolerance;
  lower_bound(5) = -complementarity_tolerance;
  lower_bound(6) = 0;
  return lower_bound;
}

Eigen::Matrix<double, 11, 1> GetConstraintUpperBound(
    double complementarity_tolerance) {
  Eigen::Matrix<double, 11, 1> upper_bound =
      Eigen::Matrix<double, 11, 1>::Zero();
  upper_bound(3) = complementarity_tolerance;
  upper_bound(4) = complementarity_tolerance;
  upper_bound(5) = complementarity_tolerance;
  upper_bound(6) = kInf;
  return upper_bound;
}
}  // namespace

SlidingFrictionComplementarityNonlinearConstraint::
    SlidingFrictionComplementarityNonlinearConstraint(
        const ContactWrenchEvaluator* const contact_wrench_evaluator,
        double complementarity_tolerance)
    : solvers::Constraint(
          11,
          contact_wrench_evaluator->plant().num_positions() +
              contact_wrench_evaluator->plant().num_velocities() +
              contact_wrench_evaluator->num_lambda() + 7,
          GetConstraintLowerBound(complementarity_tolerance),
          GetConstraintUpperBound(complementarity_tolerance),
          "sliding_friction_complementarity_constraint"),
      contact_wrench_evaluator_{contact_wrench_evaluator},
      c_var_{"c"} {}

std::vector<std::pair<int, int>>
SlidingFrictionComplementarityNonlinearConstraint::
    GetConstraintSparsityPattern() const {
  const auto& plant = contact_wrench_evaluator_->plant();
  // Set the sparsity pattern.
  // gradient_sparsity_pattern contains all the entries in the gradient that can
  // be non-zero.
  std::vector<std::pair<int, int>> gradient_sparsity_pattern;
  gradient_sparsity_pattern.reserve(
      11 * plant.num_positions() + 6 * plant.num_velocities() +
      3 * contact_wrench_evaluator_->num_lambda() + 24);
  Eigen::VectorXi x_indices =
      Eigen::VectorXi::LinSpaced(num_vars(), 0, num_vars() - 1);

  Eigen::VectorXi q_indices, v_indices, lambda_indices;
  Eigen::Vector3i f_static_indices, f_sliding_indices;
  int c_index;
  DecomposeX<int>(x_indices, &q_indices, &v_indices, &lambda_indices,
                  &f_static_indices, &f_sliding_indices, &c_index);
  // If the i'th row of the constraint depends on some variables with the given
  // variable indices, then fill in gradient_sparsity_pattern.
  auto set_gradient_dependent_variables =
      [&gradient_sparsity_pattern](
          int constraint_row_index,
          const Eigen::Ref<const Eigen::VectorXi>& var_indices) {
        for (int i = 0; i < var_indices.rows(); ++i) {
          gradient_sparsity_pattern.emplace_back(constraint_row_index,
                                                 var_indices(i));
        }
      };
  // Constraint (1) depends on q, lambda, f_static and f_sliding.
  for (int i = 0; i < 3; ++i) {
    set_gradient_dependent_variables(i, q_indices);
    set_gradient_dependent_variables(i, lambda_indices);
    gradient_sparsity_pattern.emplace_back(i, f_static_indices(i));
    gradient_sparsity_pattern.emplace_back(i, f_sliding_indices(i));
  }
  // Constraint (2) depends on  q, v, f_static.
  for (int i = 3; i < 6; ++i) {
    set_gradient_dependent_variables(i, q_indices);
    set_gradient_dependent_variables(i, v_indices);
    set_gradient_dependent_variables(i, f_static_indices);
  }
  // Constraint (3) depends on q and f_sliding.
  for (int i = 6; i < 8; ++i) {
    set_gradient_dependent_variables(i, q_indices);
    set_gradient_dependent_variables(i, f_sliding_indices);
  }
  // Constraint (4) depends on q, v, f_sliding, c
  for (int i = 8; i < 11; ++i) {
    set_gradient_dependent_variables(i, q_indices);
    set_gradient_dependent_variables(i, v_indices);
    set_gradient_dependent_variables(i, f_sliding_indices);
    gradient_sparsity_pattern.emplace_back(i, c_index);
  }
  return gradient_sparsity_pattern;
}

void SlidingFrictionComplementarityNonlinearConstraint::
    UpdateComplementarityTolerance(double complementarity_tolerance) {
  UpdateLowerBound(GetConstraintLowerBound(complementarity_tolerance));
  UpdateUpperBound(GetConstraintUpperBound(complementarity_tolerance));
}

void SlidingFrictionComplementarityNonlinearConstraint::DoEval(
    const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::VectorXd* y) const {
  AutoDiffVecXd y_autodiff;
  Eval(x.cast<AutoDiffXd>(), &y_autodiff);
  *y = math::ExtractValue(y_autodiff);
}

void SlidingFrictionComplementarityNonlinearConstraint::DoEval(
    const Eigen::Ref<const AutoDiffVecXd>& x, AutoDiffVecXd* y) const {
  y->resize(num_constraints());
  AutoDiffVecXd q, v, lambda;
  Vector3<AutoDiffXd> f_static, f_sliding;
  AutoDiffXd c;
  DecomposeX(x, &q, &v, &lambda, &f_static, &f_sliding, &c);
  // Update context with q and v.
  const auto& plant = contact_wrench_evaluator_->plant();
  systems::Context<AutoDiffXd>& context =
      const_cast<systems::Context<AutoDiffXd>&>(
          contact_wrench_evaluator_->context());
  if (!internal::AreAutoDiffVecXdEqual(q, plant.GetPositions(context))) {
    plant.SetPositions(&context, q);
  }
  if (!internal::AreAutoDiffVecXdEqual(v, plant.GetVelocities(context))) {
    plant.SetVelocities(&context, v);
  }

  // Compute the contact wrench F_AB_W
  AutoDiffVecXd F_AB_W;
  contact_wrench_evaluator_->Eval(
      contact_wrench_evaluator_->ComposeVariableValues(context, lambda),
      &F_AB_W);
  Vector3<AutoDiffXd> f_AB_W = F_AB_W.tail<3>();

  // constraint 1: f_AB_W = f_static + f_sliding
  y->head<3>() = f_AB_W - f_static - f_sliding;

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
  const std::vector<geometry::SignedDistancePair<AutoDiffXd>>
      signed_distance_pairs =
          query_object.ComputeSignedDistancePairwiseClosestPoints();
  bool found_geometry_pair = false;
  for (const auto& signed_distance_pair : signed_distance_pairs) {
    if (signed_distance_pair.id_A ==
            contact_wrench_evaluator_->geometry_id_pair().first() &&
        signed_distance_pair.id_B ==
            contact_wrench_evaluator_->geometry_id_pair().second()) {
      found_geometry_pair = true;

      const geometry::SceneGraphInspector<AutoDiffXd>& inspector =
          query_object.inspector();

      // Compute the friction.
      const geometry::ProximityProperties& geometryA_props =
          *inspector.GetProximityProperties(signed_distance_pair.id_A);
      const geometry::ProximityProperties& geometryB_props =
          *inspector.GetProximityProperties(signed_distance_pair.id_B);

      const CoulombFriction<double>& geometryA_friction =
          geometryA_props.GetProperty<CoulombFriction<double>>(
              "material", "coulomb_friction");
      const CoulombFriction<double>& geometryB_friction =
          geometryB_props.GetProperty<CoulombFriction<double>>(
              "material", "coulomb_friction");

      CoulombFriction<double> combined_friction =
          CalcContactFrictionFromSurfaceProperties(geometryA_friction,
                                                   geometryB_friction);

      const geometry::FrameId frame_A_id =
          inspector.GetFrameId(signed_distance_pair.id_A);
      const geometry::FrameId frame_B_id =
          inspector.GetFrameId(signed_distance_pair.id_B);
      const Frame<AutoDiffXd>& frameA =
          plant.GetBodyFromFrameId(frame_A_id)->body_frame();
      const Frame<AutoDiffXd>& frameB =
          plant.GetBodyFromFrameId(frame_B_id)->body_frame();

      const SpatialVelocity<AutoDiffXd> V_AB_W =
          frameB.CalcSpatialVelocity(context, frameA, plant.world_frame());

      // We use Bg to mean the geometry frame attached to body B, and Bb to mean
      // the body frame of body B.
      const Vector3<AutoDiffXd>& p_BgCb = signed_distance_pair.p_BCb;
      const Vector3<AutoDiffXd> p_BbCb =
          inspector.GetPoseInFrame(signed_distance_pair.id_B)
              .template cast<AutoDiffXd>() *
          p_BgCb;
      const Vector3<AutoDiffXd> p_BbCb_W =
          plant.CalcRelativeTransform(context, plant.world_frame(), frameB)
              .rotation() *
          p_BbCb;

      // v_ACb_W is the sliding velocity of witness point Cb measured from the
      // geometry A, expressed in the world frame.
      const Vector3<AutoDiffXd> v_ACb_W =
          V_AB_W.Shift(p_BbCb_W).translational();

      // We need to project the sliding velocity v_ACb_W to the tangential
      // plane. To do so, we first find the normal vector nhat_BA_W of that
      // tangential plane
      const auto& nhat_BA_W = signed_distance_pair.nhat_BA_W;
      const Vector3<AutoDiffXd> v_sliding_ACb_W =
          (Eigen::Matrix3d::Identity() - nhat_BA_W * nhat_BA_W.transpose()) *
          v_ACb_W;

      // Impose the constraint (2), -eps <= v_sliding_tangential *
      // f_static_normal <= eps
      for (int i = 0; i < 3; ++i) {
        (*y)(3 + i) = v_sliding_ACb_W(i) * f_static.dot(nhat_BA_W);
      }

      // Impose constraint(3), μ * f_sliding_normal = |f_sliding_tangential|
      // This constraint can be reformulated as
      // f_slidingᵀ * -nhat_BA_W >= 0
      // f_slidingᵀ * (I - (μ²+1) * nhat_BA_W * nhat_BA_Wᵀ) * f_sliding = 0
      (*y)(6) = f_sliding.dot(-nhat_BA_W);
      (*y)(7) =
          f_sliding.dot((Eigen::Matrix3d::Identity() -
                         (pow(combined_friction.dynamic_friction(), 2) + 1) *
                             nhat_BA_W * nhat_BA_W.transpose()) *
                        f_sliding);

      // Impose constraint (4) f_sliding_tangential  + c * v_sliding_tangential
      // = 0.
      const Vector3<AutoDiffXd> f_sliding_tangential =
          (Eigen::Matrix3d::Identity() - nhat_BA_W * nhat_BA_W.transpose()) *
          f_sliding;
      y->segment<3>(8) = f_sliding_tangential + c * v_sliding_ACb_W;

      break;
    }
  }
  if (!found_geometry_pair) {
    throw std::runtime_error(
        "SlidingFrictionComplementarityNonlinearConstraint: the input "
        "contact_wrench_evaluator contains a pair of geometry that has not "
        "been registered to the SceneGraph for distance computation.");
  }
}

void SlidingFrictionComplementarityNonlinearConstraint::DoEval(
    const Eigen::Ref<const VectorX<symbolic::Variable>>&,
    VectorX<symbolic::Expression>*) const {
  throw std::runtime_error(
      "SlidingFrictionComplementarityNonlinearConstraint: Eval doesn't support "
      "symbolic variables yet.");
}

solvers::Binding<internal::SlidingFrictionComplementarityNonlinearConstraint>
AddSlidingFrictionComplementarityConstraint(
    const ContactWrenchEvaluator* contact_wrench_evaluator,
    double complementarity_tolerance,
    const Eigen::Ref<const VectorX<symbolic::Variable>>& q_vars,
    const Eigen::Ref<const VectorX<symbolic::Variable>>& v_vars,
    const Eigen::Ref<const VectorX<symbolic::Variable>>& lambda_vars,
    solvers::MathematicalProgram* prog) {
  auto constraint = std::make_shared<
      internal::SlidingFrictionComplementarityNonlinearConstraint>(
      contact_wrench_evaluator, complementarity_tolerance);
  const auto f_static = prog->NewContinuousVariables<3>("f_static");
  const auto f_sliding = prog->NewContinuousVariables<3>("f_sliding");
  const auto c_var = prog->NewContinuousVariables<1>(
      "sliding_friction_complementarity_slack_c")(0);
  prog->AddBoundingBoxConstraint(0, kInf, c_var);
  VectorX<symbolic::Variable> bound_vars(constraint->num_vars());
  constraint->ComposeX<symbolic::Variable>(
      q_vars, v_vars, lambda_vars, f_static, f_sliding, c_var, &bound_vars);
  prog->AddConstraint(constraint, bound_vars);
  return solvers::Binding<
      internal::SlidingFrictionComplementarityNonlinearConstraint>(constraint,
                                                                   bound_vars);
}
}  // namespace internal

std::pair<solvers::Binding<
              internal::SlidingFrictionComplementarityNonlinearConstraint>,
          solvers::Binding<StaticFrictionConeConstraint>>
AddSlidingFrictionComplementarityExplicitContactConstraint(
    const ContactWrenchEvaluator* contact_wrench_evaluator,
    double complementarity_tolerance,
    const Eigen::Ref<const VectorX<symbolic::Variable>>& q_vars,
    const Eigen::Ref<const VectorX<symbolic::Variable>>& v_vars,
    const Eigen::Ref<const VectorX<symbolic::Variable>>& lambda_vars,
    solvers::MathematicalProgram* prog) {
  const auto sliding_friction_complementarity_constraint =
      internal::AddSlidingFrictionComplementarityConstraint(
          contact_wrench_evaluator, complementarity_tolerance, q_vars, v_vars,
          lambda_vars, prog);
  solvers::Binding<StaticFrictionConeConstraint>
      static_friction_cone_constraint(
          std::make_shared<StaticFrictionConeConstraint>(
              contact_wrench_evaluator),
          {q_vars, lambda_vars});
  prog->AddConstraint(static_friction_cone_constraint);
  return std::make_pair(sliding_friction_complementarity_constraint,
                        static_friction_cone_constraint);
}

std::pair<solvers::Binding<
              internal::SlidingFrictionComplementarityNonlinearConstraint>,
          solvers::Binding<
              internal::StaticFrictionConeComplementarityNonlinearConstraint>>
AddSlidingFrictionComplementarityImplicitContactConstraint(
    const ContactWrenchEvaluator* contact_wrench_evaluator,
    double complementarity_tolerance,
    const Eigen::Ref<const VectorX<symbolic::Variable>>& q_vars,
    const Eigen::Ref<const VectorX<symbolic::Variable>>& v_vars,
    const Eigen::Ref<const VectorX<symbolic::Variable>>& lambda_vars,
    solvers::MathematicalProgram* prog) {
  const auto sliding_friction_complementarity_constraint =
      internal::AddSlidingFrictionComplementarityConstraint(
          contact_wrench_evaluator, complementarity_tolerance, q_vars, v_vars,
          lambda_vars, prog);
  const auto static_friction_complementarity_constraint =
      AddStaticFrictionConeComplementarityConstraint(contact_wrench_evaluator,
                                                     complementarity_tolerance,
                                                     q_vars, lambda_vars, prog);
  return std::make_pair(sliding_friction_complementarity_constraint,
                        static_friction_complementarity_constraint);
}
}  // namespace multibody
}  // namespace drake
