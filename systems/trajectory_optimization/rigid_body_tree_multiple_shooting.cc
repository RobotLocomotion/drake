#include "drake/systems/trajectory_optimization/rigid_body_tree_multiple_shooting.h"

#include <string>
#include <utility>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/systems/trajectory_optimization/rigid_body_tree_multiple_shooting_internal.h"

namespace drake {
namespace systems {
namespace trajectory_optimization {

// This evaluator computes the generalized constraint force Jᵀλ.
GeneralizedConstraintForceEvaluator::GeneralizedConstraintForceEvaluator(
    const RigidBodyTree<double>& tree, int num_lambda,
    std::shared_ptr<KinematicsCacheWithVHelper<AutoDiffXd>> kinematics_helper)
    : EvaluatorBase(tree.get_num_velocities(),
                    tree.get_num_positions() + num_lambda,
                    "generalized constraint force"),
      tree_{&tree},
      num_lambda_(num_lambda),
      kinematics_helper_{kinematics_helper} {}

void GeneralizedConstraintForceEvaluator::DoEval(
    const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::VectorXd& y) const {
  AutoDiffVecXd y_t;
  Eval(math::initializeAutoDiff(x), y_t);
  y = math::autoDiffToValueMatrix(y_t);
}

void GeneralizedConstraintForceEvaluator::DoEval(
    const Eigen::Ref<const AutoDiffVecXd>& x, AutoDiffVecXd& y) const {
  // x contains q and λ
  DRAKE_ASSERT(x.rows() == tree_->get_num_positions() + num_lambda_);
  const auto q = x.head(tree_->get_num_positions());
  const auto lambda = x.tail(num_lambda_);

  auto kinsol = kinematics_helper_->UpdateKinematics(q);
  const auto J_position_constraint =
      tree_->positionConstraintsJacobian(kinsol, false);
  const int num_position_constraint_lambda = tree_->getNumPositionConstraints();
  const auto position_constraint_lambda =
      lambda.head(num_position_constraint_lambda);
  y = J_position_constraint.transpose() * position_constraint_lambda;
  // If there are more constraint, such as foot above the ground, then you
  // should compute the Jacobian of the foot toe, multiply the transpose of
  // this Jacobian with the ground contact force, and add the product to y.
}

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

DirectTranscriptionConstraint::DirectTranscriptionConstraint(
    const RigidBodyTree<double>& tree, 
    std::shared_ptr<KinematicsCacheWithVHelper<AutoDiffXd>> kinematics_helper,
    std::unique_ptr<GeneralizedConstraintForceEvaluator>
        generalized_constraint_force_evaluator)
    : Constraint(tree.get_num_positions() + tree.get_num_velocities(),
                 1 + 2 * tree.get_num_positions() +
                     2 * tree.get_num_velocities() + tree.get_num_actuators() +
                     generalized_constraint_force_evaluator->num_lambda(),
                 Eigen::VectorXd::Zero(tree.get_num_positions() +
                                       tree.get_num_velocities()),
                 Eigen::VectorXd::Zero(tree.get_num_positions() +
                                       tree.get_num_velocities())),
      tree_(&tree),
      num_positions_{tree.get_num_positions()},
      num_velocities_{tree.get_num_velocities()},
      num_actuators_{tree.get_num_actuators()},
      num_lambda_{generalized_constraint_force_evaluator->num_lambda()},
      kinematics_helper1_{kinematics_helper},
      generalized_constraint_force_evaluator_(
          std::move(generalized_constraint_force_evaluator)) {
  DRAKE_THROW_UNLESS(num_positions_ == num_velocities_);
}

void DirectTranscriptionConstraint::DoEval(
    const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::VectorXd& y) const {
  AutoDiffVecXd y_t;
  Eval(math::initializeAutoDiff(x), y_t);
  y = math::autoDiffToValueMatrix(y_t);
}

void DirectTranscriptionConstraint::DoEval(
    const Eigen::Ref<const AutoDiffVecXd>& x, AutoDiffVecXd& y) const {
  DRAKE_ASSERT(x.size() == num_vars());

  int x_count = 0;
  // A lambda expression to take num_element entreis from x, in a certain
  // order.
  auto x_segment = [x, &x_count](int num_element) {
    x_count += num_element;
    return x.segment(x_count - num_element, num_element);
  };

  const AutoDiffXd h = x(0);
  x_count++;
  const AutoDiffVecXd q_l = x_segment(num_positions_);
  const AutoDiffVecXd v_l = x_segment(num_velocities_);
  const AutoDiffVecXd q_r = x_segment(num_positions_);
  const AutoDiffVecXd v_r = x_segment(num_velocities_);
  const AutoDiffVecXd u_r = x_segment(num_actuators_);
  const AutoDiffVecXd lambda_r = x_segment(num_lambda_);

  auto kinsol = kinematics_helper1_->UpdateKinematics(q_r, v_r);

  y.resize(num_constraints());

  // By using backward Euler integration, the constraint is
  // qᵣ - qₗ = q̇ᵣ*h
  // Mᵣ(vᵣ - vₗ) = (B*uᵣ + Jᵣᵀ*λᵣ -c(qᵣ, vᵣ))h
  // We assume here q̇ᵣ = vᵣ
  // TODO(hongkai.dai): compute qdot_r from q_r and v_r.
  y.head(num_positions_) = q_r - q_l - v_r * h;

  const auto M = tree_->massMatrix(kinsol);

  // Compute the Coriolis force, centripedal force, etc.
  const typename RigidBodyTree<AutoDiffXd>::BodyToWrenchMap
      no_external_wrenches;
  const auto c = tree_->dynamicsBiasTerm(kinsol, no_external_wrenches);

  // Compute Jᵀλ
  AutoDiffVecXd q_lambda(num_positions_ + num_lambda_);
  q_lambda << q_r, lambda_r;
  AutoDiffVecXd generalized_constraint_force(num_velocities_);
  generalized_constraint_force_evaluator_->Eval(q_lambda,
                                                generalized_constraint_force);

  y.tail(num_velocities_) =
      M * (v_r - v_l) - (tree_->B * u_r + generalized_constraint_force - c) * h;
}

RigidBodyTreeMultipleShooting::RigidBodyTreeMultipleShooting(
    const RigidBodyTree<double>& tree, const std::vector<int>& num_lambdas,
    int num_time_samples, double minimum_timestep, double maximum_timestep)
    : MultipleShooting(tree.get_num_actuators(),
                       tree.get_num_positions() + tree.get_num_velocities(),
                       num_time_samples, minimum_timestep, maximum_timestep),
      tree_{&tree},
      num_positions_{tree.get_num_positions()},
      num_velocities_{tree.get_num_velocities()},
      num_lambdas_{num_lambdas} {
  if (static_cast<int>(num_lambdas.size()) != num_time_samples) {
    std::ostringstream oss;
    oss << "lambda should be a vector of size " << num_time_samples << "\n";
    throw std::runtime_error(oss.str());
  }
  // For each knot, we will need to impose a transcription/collocation
  // constraint. Each of these constraints require us caching some
  // kinematics info.
  kinematics_with_v_helpers_.resize(num_time_samples);
  for (int i = 0; i < num_time_samples; ++i) {
    kinematics_with_v_helpers_[i] =
        std::make_shared<KinematicsCacheWithVHelper<AutoDiffXd>>(*tree_);
  }

  q_vars_.resize(num_positions_, N());
  v_vars_.resize(num_velocities_, N());
  lambda_vars_.resize(N());
  for (int i = 0; i < N(); ++i) {
    q_vars_.col(i) = x_vars().segment(num_states() * i, num_positions_);
    v_vars_.col(i) =
        x_vars().segment(num_states() * i + num_positions_, num_velocities_);
    const std::string lambda_name = "lambda[" + std::to_string(i) + "]";
    lambda_vars_[i] = NewContinuousVariables(num_lambdas_[i], lambda_name);
  }
  DoAddCollocationOrTranscriptionConstraint();
}

void RigidBodyTreeMultipleShooting::
    DoAddCollocationOrTranscriptionConstraint() {
  for (int i = 0; i < N() - 1; ++i) {
    auto generalized_constraint_force_evaluator =
        std::make_unique<GeneralizedConstraintForceEvaluator>(
            *tree_, num_lambdas_[i + 1], kinematics_with_v_helpers_[i + 1]);
    auto transcription_cnstr = std::make_shared<DirectTranscriptionConstraint>(
        *tree_, kinematics_with_v_helpers_[i + 1],
        std::move(generalized_constraint_force_evaluator));
    AddConstraint(transcription_cnstr,
                  transcription_cnstr->CompositeEvalInput(
                      h_vars()(i), q_vars_.col(i), v_vars_.col(i),
                      q_vars_.col(i + 1), v_vars_.col(i + 1),
                      u_vars().segment((i + 1) * num_inputs(), num_inputs()),
                      lambda_vars_[i + 1]));
  }
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

PiecewisePolynomialTrajectory
RigidBodyTreeMultipleShooting::ReconstructStateTrajectory() const {
  Eigen::VectorXd times = GetSampleTimes();
  std::vector<double> times_vec(N());
  std::vector<Eigen::MatrixXd> states(N());

  for (int i = 0; i < N(); ++i) {
    times_vec[i] = times(i);
    states[i] = GetSolution(state(i));
  }
  return PiecewisePolynomialTrajectory(
      PiecewisePolynomial<double>::FirstOrderHold(times_vec, states));
}

PiecewisePolynomialTrajectory
RigidBodyTreeMultipleShooting::ReconstructInputTrajectory() const {
  Eigen::VectorXd times = GetSampleTimes();
  std::vector<double> times_vec(N());
  std::vector<Eigen::MatrixXd> inputs(N());

  for (int i = 0; i < N(); ++i) {
    times_vec[i] = times(i);
    inputs[i] = GetSolution(input(i));
  }
  return PiecewisePolynomialTrajectory(
      PiecewisePolynomial<double>::ZeroOrderHold(times_vec, inputs));
}

}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace drake
