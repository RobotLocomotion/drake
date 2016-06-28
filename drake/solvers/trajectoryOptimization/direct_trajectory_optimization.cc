#include "direct_trajectory_optimization.h"

#include <iostream>
#include <Eigen/Core>

using namespace std;

namespace drake {
namespace solvers {

/**
 *  DirectTrajectoryOptimization
 *
 *  N number of timesteps
 *  h timesteps (there are N-1 of these)
 *  x state
 *  u control input
 */
DirectTrajectoryOptimization::DirectTrajectoryOptimization(
    const int numInputs, const int numStates,
    const size_t numTimeSamples /* N */, const int trajectoryTimeLowerBound,
    const int trajectoryTimeUpperBound)
    : numInputs_(numInputs),
      numStates_(numStates),
      N(numTimeSamples),
      h_vars_(optProblem_.AddContinuousVariables(N - 1, "h")) {
  // Construct total time linear constraint.
  // TODO(tri-lucy) add case for all timesteps independent (if needed).
  MatrixXd id_zero(N - 2, N - 1);
  id_zero << MatrixXd::Identity(N - 2, N - 2), MatrixXd::Zero(N - 2, 1);
  MatrixXd zero_id(N - 2, N - 1);
  zero_id << MatrixXd::Zero(N - 2, 1), MatrixXd::Identity(N - 2, N - 2);
  MatrixXd a_time(N - 1, N - 1);
  a_time << MatrixXd::Ones(1, N - 1), id_zero - zero_id;
  //  cout << a_time << endl;
  VectorXd lower(N - 1);
  lower << trajectoryTimeLowerBound, MatrixXd::Zero(N - 2, 1);
  VectorXd upper(N - 1);
  upper << trajectoryTimeUpperBound, MatrixXd::Zero(N - 2, 1);
  optProblem_.AddLinearConstraint(a_time, lower, upper, {h_vars_});

  // Ensure that all h values are non-negative.
  // TODO(tri-lucy): add bounding box constraint. See testOpt Problem.cpp

  // Create constraints for dynamics and add them.

  // Add control inputs (upper and lower bounds) as bounding box constraints.
  // TODO(tri-lucy): also add if !inf (see Matlab).
  /*
    VectorXd lower(N - 1);
    lower << trajectoryTimeLowerBound; // TODO add more here.
    VectorXd upper(N - 1);
    upper << trajectoryTimeUpperBound;
    optProblem_.AddBoundingBoxConstraint(lower, upper);
  */
}

}  // solvers
}  // drake
