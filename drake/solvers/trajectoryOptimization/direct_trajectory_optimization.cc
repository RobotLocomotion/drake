#include "direct_trajectory_optimization.h"

namespace drake {
namespace solvers {
namespace {
VectorXd vector_diff(VectorXd& vec) {
  return vec.tail(vec.size() - 1) - vec.head(vec.size() - 1);
}
}

/**
 *  DirectTrajectoryOptimization
 *
 *  For readability of long lines, these single-letter variables names are
 *  sometimes used:
 *
 *  N number of timesteps/samples
 *
 *  h timesteps (there are N-1 of these)
 *
 *  x state
 *
 *  u control input
 */
DirectTrajectoryOptimization::DirectTrajectoryOptimization(
    const int num_inputs, const int num_states,
    const int num_time_samples,
    const int trajectory_time_lower_bound,
    const int trajectory_time_upper_bound)
    : num_inputs_(num_inputs),
      num_states_(num_states),
      N_(num_time_samples),
      h_vars_(opt_problem_.AddContinuousVariables(N_ - 1, "h")),
      u_vars_(opt_problem_.AddContinuousVariables(num_inputs * N_, "u")),
      x_vars_(opt_problem_.AddContinuousVariables(num_states * N_, "x")) {
  // Construct total time linear constraint.
  // TODO(Lucy-tri) add case for all timesteps independent (if needed).
  MatrixXd id_zero(N_ - 2, N_ - 1);
  id_zero << MatrixXd::Identity(N_ - 2, N_ - 2), MatrixXd::Zero(N_ - 2, 1);
  MatrixXd zero_id(N_ - 2, N_ - 1);
  zero_id << MatrixXd::Zero(N_ - 2, 1), MatrixXd::Identity(N_ - 2, N_ - 2);
  MatrixXd a_time(N_ - 1, N_ - 1);
  a_time << MatrixXd::Ones(1, N_ - 1), id_zero - zero_id;

  VectorXd lower(N_ - 1);
  lower << trajectory_time_lower_bound, MatrixXd::Zero(N_ - 2, 1);
  VectorXd upper(N_ - 1);
  upper << trajectory_time_upper_bound, MatrixXd::Zero(N_ - 2, 1);
  opt_problem_.AddLinearConstraint(a_time, lower, upper, {h_vars_});

  // Ensure that all h values are non-negative.
  VectorXd all_inf(N_ - 1);
  all_inf.fill(std::numeric_limits<double>::infinity());
  opt_problem_.AddBoundingBoxConstraint(MatrixXd::Zero(N_ - 1, 1), all_inf,
                                        {h_vars_});

  // TODO(Lucy-tri) Create constraints for dynamics and add them.
  // Matlab: obj.addDynamicConstraints();

  // TODO(Lucy-tri) Add control inputs (upper and lower bounds) as bounding box
  // constraints.
}

void DirectTrajectoryOptimization::GetInitialVars(
    int t_init_in, const PiecewisePolynomial<double>& traj_init_u,
    const PiecewisePolynomial<double>& traj_init_x) {
  VectorXd t_init{VectorXd::LinSpaced(N_, 0, t_init_in)};
  opt_problem_.SetInitialGuess(h_vars_, vector_diff(t_init));

  VectorXd guess_u(u_vars_.size());
  if (traj_init_u.empty()) {
    guess_u = 0.01 * VectorXd::Random(u_vars_.size());
  } else {
    for (int t = 0; t < N_; ++t) {
      guess_u.segment(num_inputs_ * t, num_inputs_) =
          traj_init_u.value(t_init[t]);
    }
  }
  opt_problem_.SetInitialGuess(u_vars_, guess_u);

  DRAKE_ASSERT(!traj_init_x.empty());  // TODO(Lucy-tri) see below.
  VectorXd guess_x(x_vars_.size());
  if (traj_init_x.empty()) {
    // TODO(Lucy-tri) Do what DirectTrajectoryOptimization.m does.
  } else {
    for (int t = 0; t < N_; ++t) {
      guess_x.segment(num_states_ * t, num_states_) =
          traj_init_x.value(t_init[t]);
    }
  }
  opt_problem_.SetInitialGuess(x_vars_, guess_x);
}

// TODO(Lucy-tri) add optional x_indices. Do: time_index as cell array.
void DirectTrajectoryOptimization::AddStateConstraint(
    const Constraint& constraint, const int time_index) {}

SolutionResult DirectTrajectoryOptimization::SolveTraj(
    int t_init, const PiecewisePolynomial<double>& traj_init_u,
    const PiecewisePolynomial<double>& traj_init_x) {
  GetInitialVars(t_init, traj_init_u, traj_init_x);
  SolutionResult result = opt_problem_.Solve();
  // TODO(Lucy-tri) Reconstruct the state and input trajectories.
  return result;
}

}  // solvers
}  // drake
