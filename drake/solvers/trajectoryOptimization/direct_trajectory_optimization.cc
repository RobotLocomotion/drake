#include "direct_trajectory_optimization.h"

#include <iostream>
#include <Eigen/Core>

using Eigen::MatrixXd;

using namespace std;

namespace drake {
namespace solvers {
namespace {

void setupVariables(OptimizationProblem optProblem, size_t N, int numStates,
                    int numInputs) {
  int nH = N - 1;
  int nX = numStates;
  int nU = numInputs;
  //  int numVars = nH + N * (nX + nU);

  //  DecisionVariableView h_vars =
  optProblem.AddContinuousVariables(nH, "h");
  //  DecisionVariableView x_vars =
  optProblem.AddContinuousVariables(N * nX, "x");
  //  DecisionVariableView u_vars =
  optProblem.AddContinuousVariables(N * nU, "u");
}
}  // unnamed namespace

DirectTrajectoryOptimization::DirectTrajectoryOptimization(
    const int numInputs, const int numStates,
    const size_t numTimeSamples /* N */, const int trajectoryTimeLowerBound,
    const int trajectoryTimeUpperBound)
    : numInputs_(numInputs), numStates_(numStates), N(numTimeSamples) {
  setupVariables(optProblem_, N, numStates_, numInputs_);

  MatrixXd a_time(N - 2, N - 1);
  cout << "Identity + Zero:" << endl;
  a_time << MatrixXd::Identity(N - 2, N - 2), MatrixXd::Zero(N - 2, 1);
  cout << a_time << endl;

  cout << "Zero + Identity:" << endl;
  MatrixXd a_time2(N - 2, N - 1);
  a_time2 << MatrixXd::Zero(N - 2, 1), MatrixXd::Identity(N - 2, N - 2);
  cout << a_time2 << endl;

  cout << "Difference:" << endl;
  a_time -= a_time2;
  cout << a_time << endl;

  cout << "Final:" << endl;
  MatrixXd a_time3(N - 1, N - 1);
  a_time3 << MatrixXd::Ones(1, N - 1), a_time;
  cout << a_time3 << endl;

  // Add costs.
  // TODO(tri-lucy) create LinearConstraint from upper & lower bounds.
  // AddLinearConstraint
  // create and add bounding box constraint.
  // addDynamicConstraints
}

}  // solvers
}  // drake
