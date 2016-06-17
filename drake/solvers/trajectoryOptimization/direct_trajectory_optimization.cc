#include "direct_trajectory_optimization.h"

namespace drake {
namespace solvers {
namespace {

const DecisionVariableView setupVariables(OptimizationProblem optProblem,
                                                 size_t N, int numStates,
                                                 int numInputs) {
  int nH = N - 1;
  int nX = numStates;
  int nU = numInputs;
  int numVars = nH + N * (nX + nU);

  DecisionVariableView vars =
      optProblem.AddContinuousVariables(numVars, "h");
  // !!! Looks like we have to create 3 sets of decision vars, for h, x, u:
  // !!! Should call AddContinuousVariables 3x.
  // Add costs and constraints.
  return vars;
}
}  // unnamed namespace

DirectTrajectoryOptimization::DirectTrajectoryOptimization(
    const int numInputs, const int numStates, const size_t numTimeSamples,
    const int trajectoryTimeLowerBound, const int trajectoryTimeUpperBound)
    : numInputs_(numInputs),
      numStates_(numStates),
      numTimeSamples_(numTimeSamples) {
  DecisionVariableView vars =
      setupVariables(optProblem_, numTimeSamples_, numStates_, numInputs_);

  (void)vars;  // !!! temp to avoid warnings.

  // TODO(tri-lucy) create LinearConstraint from upper & lower bounds.
  // AddLinearConstraint
  // create and add bounding box constraint.
  // addDynamicConstraints
}

}  // solvers
}  // drake
