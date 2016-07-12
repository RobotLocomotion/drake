#pragma once

#include <cstddef>

#include <Eigen/Core>

#include "drake/drakeOptimization_export.h"
#include "drake/solvers/Optimization.h"
#include "drake/systems/trajectories/PiecewisePolynomial.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace drake {
namespace solvers {

class DRAKEOPTIMIZATION_EXPORT DirectTrajectoryOptimization {
 public:
  DirectTrajectoryOptimization(const int num_inputs, const int num_states,
                               const size_t num_time_samples,
                               const int trajectory_time_lower_bound,
                               const int trajectory_time_upper_bound);
  void GetInitialVars(int t_init_in, 
                      const PiecewisePolynomial<double>& traj_init_u,
                      const PiecewisePolynomial<double>& traj_init_x);
  void AddStateConstraint(const Constraint& constraint, const int time_index);
  // TODO(lucy-tri) add options params
 private:
  const int num_inputs_;
  const int num_states_;
  const int num_vars_;
  const size_t N;  // Number of time samples

  OptimizationProblem opt_problem_;
  DecisionVariableView h_vars_;
  DecisionVariableView u_vars_;
  DecisionVariableView x_vars_;
};

}  // solvers
}  // drake
