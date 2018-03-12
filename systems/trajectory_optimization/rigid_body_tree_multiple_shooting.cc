#include "drake/systems/trajectory_optimization/rigid_body_tree_multiple_shooting.h"

#include <limits>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/common/unused.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/systems/trajectory_optimization/joint_limit_constraint_force_evaluator.h"
#include "drake/systems/trajectory_optimization/position_constraint_force_evaluator.h"
#include "drake/systems/trajectory_optimization/rigid_body_tree_multiple_shooting_internal.h"

namespace drake {
namespace systems {
namespace trajectory_optimization {
using plants::KinematicsCacheHelper;
using plants::KinematicsCacheWithVHelper;

namespace {
/**
 * Add a variable to the map map_variable_to_index. If
 * error_for_duplicate_variable is set to true, then throws a runtime error
 * when the added `vars` already exist in `map_variable_to_index`.
 */
void AddVariableToMap(
    const Eigen::Ref<const solvers::VectorXDecisionVariable>& vars,
    bool error_for_duplicate_variable,
    std::unordered_map<symbolic::Variable::Id, int>* map_variable_to_index,
    solvers::VectorXDecisionVariable* aggregated_vars) {
  const int num_existing_aggregated_vars = aggregated_vars->rows();
  aggregated_vars->conservativeResize(num_existing_aggregated_vars +
                                      vars.rows());
  for (int i = 0; i < vars.rows(); ++i) {
    const auto it = map_variable_to_index->find(vars(i).get_id());
    if (it != map_variable_to_index->end()) {
      if (error_for_duplicate_variable) {
        throw std::runtime_error("This variable exists in the map already.");
      }
    } else {
      const int variable_count = map_variable_to_index->size();
      map_variable_to_index->emplace_hint(it, vars(i).get_id(), variable_count);
      (*aggregated_vars)(variable_count) = vars(i);
    }
  }
  aggregated_vars->conservativeResize(map_variable_to_index->size());
}

/**
 * This function returns the mapping, that maps all the variables bound with
 * the DirectTranscriptionConstraint to the index in the aggregated bound
 * variables. The aggregated variables is returned as the second argument.
 */
std::pair<std::unordered_map<symbolic::Variable::Id, int>,
          solvers::VectorXDecisionVariable>
GetVariableIndicesInDirectTranscriptionConstraint(
    const symbolic::Variable& h,
    const Eigen::Ref<const solvers::VectorXDecisionVariable>& q_l,
    const Eigen::Ref<const solvers::VectorXDecisionVariable>& v_l,
    const Eigen::Ref<const solvers::VectorXDecisionVariable>& q_r,
    const Eigen::Ref<const solvers::VectorXDecisionVariable>& v_r,
    const Eigen::Ref<const solvers::VectorXDecisionVariable>& u_r,
    const std::vector<solvers::Binding<GeneralizedConstraintForceEvaluator>>&
        bindings) {
  std::unordered_map<symbolic::Variable::Id, int> map_variable_to_index;
  map_variable_to_index.emplace(h.get_id(), 0);
  solvers::VectorXDecisionVariable aggregated_vars(1);
  aggregated_vars << h;
  AddVariableToMap(q_l, true, &map_variable_to_index, &aggregated_vars);
  AddVariableToMap(v_l, true, &map_variable_to_index, &aggregated_vars);
  AddVariableToMap(q_r, true, &map_variable_to_index, &aggregated_vars);
  AddVariableToMap(v_r, true, &map_variable_to_index, &aggregated_vars);
  AddVariableToMap(u_r, true, &map_variable_to_index, &aggregated_vars);
  for (const auto& binding : bindings) {
    AddVariableToMap(binding.variables(), false, &map_variable_to_index,
                     &aggregated_vars);
  }
  return std::make_pair(map_variable_to_index, aggregated_vars);
}
}  // namespace

/**
 * Implements the constraint for the backward Euler integration
 * <pre>
 * qᵣ - qₗ = q̇ᵣ*h
 * Mᵣ(vᵣ - vₗ) = (B*uᵣ + Jᵣᵀ*λᵣ -c(qᵣ, vᵣ))h
 * </pre>
 * where
 * qᵣ: The generalized position on the right knot.
 * qₗ: The generalized position on the left knot.
 * vᵣ: The generalized velocity on the right knot.
 * vₗ: The generalized velocity on the left knot.
 * uᵣ: The actuator input on the right knot.
 * Mᵣ: The inertia matrix computed from qᵣ.
 * λᵣ: The constraint force (e.g., contact force, joint limit force, etc) on the
 * right knot.
 * c(qᵣ, vᵣ): The Coriolis, gravity and centripedal force on the right knot.
 * h: The duration between the left and right knot.
 */
solvers::Binding<DirectTranscriptionConstraint>
DirectTranscriptionConstraint::Make(
    const RigidBodyTree<double>& tree,
    std::shared_ptr<plants::KinematicsCacheWithVHelper<AutoDiffXd>>
        kinematics_helper,
    const symbolic::Variable& h,
    const Eigen::Ref<const solvers::VectorXDecisionVariable>& q_l,
    const Eigen::Ref<const solvers::VectorXDecisionVariable>& v_l,
    const Eigen::Ref<const solvers::VectorXDecisionVariable>& q_r,
    const Eigen::Ref<const solvers::VectorXDecisionVariable>& v_r,
    const Eigen::Ref<const solvers::VectorXDecisionVariable>& u_r,
    const std::vector<solvers::Binding<GeneralizedConstraintForceEvaluator>>&
        constraint_force_evaluator_bindings) {
  std::unordered_map<symbolic::Variable::Id, int> map_var_to_index;
  solvers::VectorXDecisionVariable aggregated_vars;
  std::tie(map_var_to_index, aggregated_vars) =
      GetVariableIndicesInDirectTranscriptionConstraint(
          h, q_l, v_l, q_r, v_r, u_r, constraint_force_evaluator_bindings);
  std::shared_ptr<DirectTranscriptionConstraint> constraint{
      new DirectTranscriptionConstraint(tree, kinematics_helper, h, q_l, v_l,
                                        q_r, v_r, u_r, map_var_to_index,
                                        constraint_force_evaluator_bindings)};
  return solvers::Binding<DirectTranscriptionConstraint>(constraint,
                                                         aggregated_vars);
}

DirectTranscriptionConstraint::DirectTranscriptionConstraint(
    const RigidBodyTree<double>& tree,
    std::shared_ptr<KinematicsCacheWithVHelper<AutoDiffXd>> kinematics_helper,
    const symbolic::Variable& h,
    const Eigen::Ref<const solvers::VectorXDecisionVariable>& q_l,
    const Eigen::Ref<const solvers::VectorXDecisionVariable>& v_l,
    const Eigen::Ref<const solvers::VectorXDecisionVariable>& q_r,
    const Eigen::Ref<const solvers::VectorXDecisionVariable>& v_r,
    const Eigen::Ref<const solvers::VectorXDecisionVariable>& u_r,
    const std::unordered_map<symbolic::Variable::Id, int>& map_var_to_index,
    const std::vector<solvers::Binding<GeneralizedConstraintForceEvaluator>>&
        constraint_force_evaluator_bindings)
    : Constraint(
          tree.get_num_positions() + tree.get_num_velocities(),  // output size
          map_var_to_index.size(),                               // input size.
          Eigen::VectorXd::Zero(tree.get_num_positions() +
                                tree.get_num_velocities()),
          Eigen::VectorXd::Zero(tree.get_num_positions() +
                                tree.get_num_velocities())),
      tree_(&tree),
      num_positions_{tree.get_num_positions()},
      num_velocities_{tree.get_num_velocities()},
      num_actuators_{tree.get_num_actuators()},
      kinematics_helper1_{kinematics_helper} {
  // Obtain the indices of each variable vector in aggregated_variables_
  auto FindVariableIndices = [&map_var_to_index](
      const Eigen::Ref<const solvers::VectorXDecisionVariable>& vars,
      std::vector<int>* var_indices) {
    var_indices->resize(vars.rows());
    for (int i = 0; i < vars.rows(); ++i) {
      var_indices->at(i) = map_var_to_index.at(vars(i).get_id());
    }
  };
  h_index_ = map_var_to_index.at(h.get_id());
  FindVariableIndices(q_l, &q_l_indices_);
  FindVariableIndices(v_l, &v_l_indices_);
  FindVariableIndices(q_r, &q_r_indices_);
  FindVariableIndices(v_r, &v_r_indices_);
  FindVariableIndices(u_r, &u_r_indices_);
  generalized_constraint_force_evaluator_bindings_.reserve(
      constraint_force_evaluator_bindings.size());
  for (const auto& binding : constraint_force_evaluator_bindings) {
    std::vector<int> evaluator_vars_indices;
    FindVariableIndices(binding.variables(), &evaluator_vars_indices);
    generalized_constraint_force_evaluator_bindings_.emplace_back(
        binding.evaluator(), evaluator_vars_indices);
  }
}

namespace {
template <typename DerivedX, typename DerivedV>
void FillInVariableValues(const Eigen::MatrixBase<DerivedX>& x,
                          const std::vector<int>& var_indices,
                          Eigen::MatrixBase<DerivedV>* var_vals) {
  DRAKE_ASSERT(var_vals->rows() == static_cast<int>(var_indices.size()));
  for (int i = 0; i < var_vals->rows(); ++i) {
    (*var_vals)(i) = x(var_indices[i]);
  }
}
}  // namespace

void DirectTranscriptionConstraint::DoEval(
    const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::VectorXd& y) const {
  AutoDiffVecXd y_t;
  Eval(math::initializeAutoDiff(x), y_t);
  y = math::autoDiffToValueMatrix(y_t);
}

void DirectTranscriptionConstraint::DoEval(
    const Eigen::Ref<const AutoDiffVecXd>& x, AutoDiffVecXd& y) const {
  DRAKE_ASSERT(x.size() == num_vars());

  const AutoDiffXd h = x(h_index_);
  AutoDiffVecXd q_l(num_positions_);
  AutoDiffVecXd v_l(num_velocities_);
  AutoDiffVecXd q_r(num_positions_);
  AutoDiffVecXd v_r(num_velocities_);
  AutoDiffVecXd u_r(tree_->get_num_actuators());
  FillInVariableValues(x, q_l_indices_, &q_l);
  FillInVariableValues(x, v_l_indices_, &v_l);
  FillInVariableValues(x, q_r_indices_, &q_r);
  FillInVariableValues(x, v_r_indices_, &v_r);
  FillInVariableValues(x, u_r_indices_, &u_r);

  auto kinsol = kinematics_helper1_->UpdateKinematics(q_r, v_r);

  y.resize(num_constraints());

  // By using backward Euler integration, the constraint is
  // qᵣ - qₗ = q̇ᵣ*h
  // Mᵣ(vᵣ - vₗ) = (B*uᵣ + Jᵣᵀ*λᵣ -c(qᵣ, vᵣ))h
  const MatrixX<AutoDiffXd> map_v_to_qdot =
      RigidBodyTree<double>::GetVelocityToQDotMapping(kinsol);
  const AutoDiffVecXd qdot_r = map_v_to_qdot * v_r;
  // TODO(hongkai.dai): Project qdot_r to the constraint manifold (for example,
  // if q contains unit quaternion, and we need to project this backward Euler
  // integration on the unit quaternion manifold.)
  y.head(num_positions_) = q_r - q_l - qdot_r * h;

  const auto M = tree_->massMatrix(kinsol);

  // Compute the Coriolis force, centripedal force, etc.
  const typename RigidBodyTree<AutoDiffXd>::BodyToWrenchMap
      no_external_wrenches;
  const auto c = tree_->dynamicsBiasTerm(kinsol, no_external_wrenches);

  // Compute Jᵀλ
  AutoDiffVecXd total_generalized_constraint_force(num_velocities_);
  total_generalized_constraint_force.setZero();
  for (const auto& binding : generalized_constraint_force_evaluator_bindings_) {
    AutoDiffVecXd evaluator_vars(binding.first->num_vars());
    FillInVariableValues(x, binding.second, &evaluator_vars);
    AutoDiffVecXd generalized_constraint_force(num_velocities_);
    binding.first->Eval(evaluator_vars, generalized_constraint_force);
    total_generalized_constraint_force += generalized_constraint_force;
  }

  y.tail(num_velocities_) =
      M * (v_r - v_l) -
      (tree_->B * u_r + total_generalized_constraint_force - c) * h;
}

namespace {
// This class encodes the complementarity constraint on the joint limit
// constraint force λ, and the joint q.
// The constraints are
// (qᵤ - q) * λᵤ = 0
// (q - qₗ) * λₗ = 0
// where qᵤ is the joint upper bound, and qₗ is the joint lower bound.
// λᵤ / λₗ are the joint limit force from upper bound and lower bound
// respectively.
class JointLimitsComplementarityConstraint : public solvers::Constraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(JointLimitsComplementarityConstraint)

  JointLimitsComplementarityConstraint(double joint_lower_bound,
                                       double joint_upper_bound)
      : solvers::Constraint(2, 3, Eigen::Vector2d::Zero(),
                            Eigen::Vector2d::Zero()),
        joint_lower_bound_(joint_lower_bound),
        joint_upper_bound_(joint_upper_bound) {}

  template <typename Scalar>
  Vector3<Scalar> ComposeEvalInputVector(
      const Scalar& q, const Scalar joint_lower_bound_force,
      const Scalar& joint_upper_bound_force) {
    return Vector3<Scalar>(q, joint_upper_bound_force, joint_lower_bound_force);
  }

 protected:
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd& y) const override {
    AutoDiffVecXd ty;
    Eval(math::initializeAutoDiff(x), ty);
    y = math::autoDiffToValueMatrix(ty);
  }

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd& y) const override {
    y.resize(2);
    y(0) = (joint_upper_bound_ - x(0)) * x(1);
    y(1) = (x(0) - joint_lower_bound_) * x(2);
  }

 private:
  const double joint_lower_bound_;
  const double joint_upper_bound_;
};
}  // namespace

RigidBodyTreeMultipleShooting::RigidBodyTreeMultipleShooting(
    const RigidBodyTree<double>& tree, int num_time_samples,
    double minimum_timestep, double maximum_timestep)
    : MultipleShooting(tree.get_num_actuators(),
                       tree.get_num_positions() + tree.get_num_velocities(),
                       num_time_samples, minimum_timestep, maximum_timestep),
      tree_{&tree},
      num_positions_{tree.get_num_positions()},
      num_velocities_{tree.get_num_velocities()},
      num_actuators_{tree.get_num_actuators()},
      constraint_force_evaluator_bindings(num_time_samples),
      kinematics_cache_helpers_(num_time_samples),
      kinematics_cache_with_v_helpers_(num_time_samples),
      position_constraint_lambda_vars_(NewContinuousVariables(
          tree.getNumPositionConstraints(), N(), "position_lambda")) {
  // For each knot, we will need to impose a transcription/collocation
  // constraint. Each of these constraints require us caching some
  // kinematics info.
  for (int i = 0; i < num_time_samples; ++i) {
    kinematics_cache_helpers_[i] =
        std::make_shared<KinematicsCacheHelper<AutoDiffXd>>(*tree_);
    kinematics_cache_with_v_helpers_[i] =
        std::make_shared<KinematicsCacheWithVHelper<AutoDiffXd>>(*tree_);
  }

  q_vars_.resize(num_positions_, N());
  v_vars_.resize(num_velocities_, N());
  for (int i = 0; i < N(); ++i) {
    q_vars_.col(i) = x_vars().segment(num_states() * i, num_positions_);
    v_vars_.col(i) =
        x_vars().segment(num_states() * i + num_positions_, num_velocities_);
  }

  // Add RigidBodyConstraint::PositionConstraint to the constraint force Jᵀλ
  // used in the dynamics for direct transcription.
  for (int i = 0; i < N(); ++i) {
    auto position_constraint_force_evaluator =
        std::make_unique<PositionConstraintForceEvaluator>(
            *tree_, kinematics_cache_helpers_[i]);
    const solvers::VectorXDecisionVariable evaluator_variables =
        position_constraint_force_evaluator->ComposeEvalInputVector(
            q_vars_.col(i), position_constraint_lambda_vars_.col(i));
    constraint_force_evaluator_bindings[i].emplace_back(
        std::move(position_constraint_force_evaluator), evaluator_variables);
  }
}

solvers::VectorDecisionVariable<2>
RigidBodyTreeMultipleShooting::AddJointLimitImplicitConstraint(
    int interval_index, int joint_position_index, int joint_velocity_index,
    double joint_lower_bound, double joint_upper_bound) {
  if (interval_index < 0 || interval_index > N() - 1) {
    throw std::runtime_error("interval_index is invalid.");
  }
  const int right_knot_index = interval_index + 1;
  const std::string lambda_name =
      "joint_" + std::to_string(joint_velocity_index) + "_limit_lambda[" +
      std::to_string(right_knot_index) + "]";
  // joint_limit_lambda[0] is lower limit force.
  // joint_limit_lambda[1] is upper limit force.
  const solvers::VectorDecisionVariable<2> joint_limit_lambda =
      NewContinuousVariables<2>(lambda_name);
  const symbolic::Variable lower_limit_force_lambda = joint_limit_lambda[0];
  const symbolic::Variable upper_limit_force_lambda = joint_limit_lambda[1];
  // λᵤ ≥ 0, λₗ ≥ 0
  AddBoundingBoxConstraint(0, std::numeric_limits<double>::infinity(),
                           joint_limit_lambda);
  // qₗ ≤ q ≤ qᵤ
  AddBoundingBoxConstraint(joint_lower_bound, joint_upper_bound,
                           q_vars_(joint_position_index, right_knot_index));
  // Adds the joint limit force to the constraint force.
  auto joint_limit_force_evaluator =
      std::make_unique<JointLimitConstraintForceEvaluator>(
          *tree_, joint_velocity_index);
  solvers::VectorDecisionVariable<2> joint_limit_force_evaluator_lambda;
  joint_limit_force_evaluator_lambda(
      JointLimitConstraintForceEvaluator::LowerLimitForceIndexInLambda()) =
      lower_limit_force_lambda;
  joint_limit_force_evaluator_lambda(
      JointLimitConstraintForceEvaluator::UpperLimitForceIndexInLambda()) =
      upper_limit_force_lambda;

  AddGeneralizedConstraintForceEvaluatorToTranscription(
      interval_index, std::move(joint_limit_force_evaluator),
      joint_limit_force_evaluator_lambda);

  // Add the complementarity constraint
  // (qᵤ - q) * λᵤ = 0
  // (q - qₗ) * λₗ = 0
  auto joint_complementary_constraint =
      std::make_shared<JointLimitsComplementarityConstraint>(joint_lower_bound,
                                                             joint_upper_bound);
  const auto joint_complementary_vars =
      joint_complementary_constraint->ComposeEvalInputVector(
          q_vars_(joint_position_index, right_knot_index),
          lower_limit_force_lambda, upper_limit_force_lambda);
  AddConstraint(joint_complementary_constraint, joint_complementary_vars);
  return joint_limit_lambda;
}

void RigidBodyTreeMultipleShooting::Compile() {
  for (int i = 0; i < N() - 1; ++i) {
    // Build direct transcription constraint
    AddConstraint(DirectTranscriptionConstraint::Make(
        *tree_, kinematics_cache_with_v_helpers_[i + 1], h_vars()(i),
        q_vars_.col(i), v_vars_.col(i), q_vars_.col(i + 1), v_vars_.col(i + 1),
        u_vars().segment((i + 1) * num_actuators_, num_actuators_),
        constraint_force_evaluator_bindings[i + 1]));
  }
}

void RigidBodyTreeMultipleShooting::
    AddGeneralizedConstraintForceEvaluatorToTranscription(
        int interval_index,
        std::unique_ptr<GeneralizedConstraintForceEvaluator> evaluator,
        const Eigen::Ref<const solvers::VectorXDecisionVariable>&
            evaluator_variables) {
  DRAKE_ASSERT(evaluator->num_vars() == evaluator_variables.rows());
  constraint_force_evaluator_bindings[interval_index + 1].emplace_back(
      std::move(evaluator), evaluator_variables);
}

void RigidBodyTreeMultipleShooting::DoAddRunningCost(
    const symbolic::Expression& g) {
  // Add the running cost ∫ g(t, x, u)
  // We discretize this continuous integration as
  // sum_{i = 0, ..., N - 2} h_i * g_{i+1}
  for (int i = 0; i < N() - 2; ++i) {
    AddCost(SubstitutePlaceholderVariables(g * h_vars()(i), i + 1));
  }
}

trajectories::PiecewisePolynomial<double>
RigidBodyTreeMultipleShooting::ReconstructStateTrajectory() const {
  Eigen::VectorXd times = GetSampleTimes();
  std::vector<double> times_vec(N());
  std::vector<Eigen::MatrixXd> states(N());

  for (int i = 0; i < N(); ++i) {
    times_vec[i] = times(i);
    states[i] = GetSolution(state(i));
  }
  return trajectories::PiecewisePolynomial<double>::FirstOrderHold(times_vec,
                                                                   states);
}

trajectories::PiecewisePolynomial<double>
RigidBodyTreeMultipleShooting::ReconstructInputTrajectory() const {
  Eigen::VectorXd times = GetSampleTimes();
  std::vector<double> times_vec(N());
  std::vector<Eigen::MatrixXd> inputs(N());

  for (int i = 0; i < N(); ++i) {
    times_vec[i] = times(i);
    inputs[i] = GetSolution(input(i));
  }
  return trajectories::PiecewisePolynomial<double>::ZeroOrderHold(times_vec,
                                                                  inputs);
}

}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace drake
