#include "direct_trajectory_optimization.h"

#include <iostream>
#include <Eigen/Core>

using namespace std;

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
 *  N number of timesteps
 *  h timesteps (there are N-1 of these)
 *  x state
 *  u control input
 */
DirectTrajectoryOptimization::DirectTrajectoryOptimization(
    const int num_inputs, const int num_states,
    const size_t num_time_samples /* N */,
    const int trajectory_time_lower_bound,
    const int trajectory_time_upper_bound)
    : num_inputs_(num_inputs),
      num_states_(num_states),
      num_vars_(num_time_samples - 1 + (num_time_samples * 
        (num_states_ + num_inputs))),
      N(num_time_samples),
      h_vars_(opt_problem_.AddContinuousVariables(N - 1, "h")),
      u_vars_(opt_problem_.AddContinuousVariables(num_inputs, "u")),
      x_vars_(opt_problem_.AddContinuousVariables(num_states, "x")) {
  // Construct total time linear constraint.
  // TODO(lucy-tri) add case for all timesteps independent (if needed).
  MatrixXd id_zero(N - 2, N - 1);
  id_zero << MatrixXd::Identity(N - 2, N - 2), MatrixXd::Zero(N - 2, 1);
  MatrixXd zero_id(N - 2, N - 1);
  zero_id << MatrixXd::Zero(N - 2, 1), MatrixXd::Identity(N - 2, N - 2);
  MatrixXd a_time(N - 1, N - 1);
  a_time << MatrixXd::Ones(1, N - 1), id_zero - zero_id;
  //  cout << a_time << endl;
  VectorXd lower(N - 1);
  lower << trajectory_time_lower_bound, MatrixXd::Zero(N - 2, 1);
  VectorXd upper(N - 1);
  upper << trajectory_time_upper_bound, MatrixXd::Zero(N - 2, 1);
  opt_problem_.AddLinearConstraint(a_time, lower, upper, {h_vars_});

  // Ensure that all h values are non-negative.
  VectorXd all_inf(N - 1);
  all_inf.fill(std::numeric_limits<double>::infinity());
  opt_problem_.AddBoundingBoxConstraint(MatrixXd::Zero(N - 1, 1), all_inf,
                                        {h_vars_});

  // TODO(lucy-tri) Create constraints for dynamics and add them.
  // Matlab: obj.addDynamicConstraints();

  // Add control inputs (upper and lower bounds) as bounding box constraints.
  if ((trajectory_time_lower_bound !=
       std::numeric_limits<double>::infinity()) ||
      (trajectory_time_upper_bound !=
       std::numeric_limits<double>::infinity())) {
    VectorXd lower_control(N);
    lower_control.fill(trajectory_time_lower_bound);
    VectorXd upper_control(N);
    upper_control.fill(trajectory_time_upper_bound);
    opt_problem_.AddBoundingBoxConstraint(lower_control, upper_control,
                                          {u_vars_});
  }
}

/**
 *  Evaluate the initial trajectories at the sampled times and construct the 
 *  nominal z0. 
 */
void DirectTrajectoryOptimization::GetInitialVars(
    int t_init_in, const PiecewisePolynomial<double>& traj_init_u,
    const PiecewisePolynomial<double>& traj_init_x) {
  VectorXd t_init{VectorXd::LinSpaced(N, 0, t_init_in)};
  opt_problem_.SetInitialGuess(h_vars_, vector_diff(t_init)); 
  // TODO return guesses for x and u vars.
//  opt_problem_.SetInitialGuess(x_vars_, traj_init_x.value(t_init)); 
}

// TODO(Lucy-tri) add optional x_indices. Do: time_index as cell array.
void DirectTrajectoryOptimization::AddStateConstraint(
    const Constraint& constraint, const int time_index) {}

}  // solvers
}  // drake
