#pragma once

#include <cstddef>

#include <Eigen/Core>

#include "drake/drakeOptimization_export.h"
#include "drake/solvers/Optimization.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace drake {
namespace solvers {

class DRAKEOPTIMIZATION_EXPORT DirectTrajectoryOptimization {
 public:
  DirectTrajectoryOptimization(const int numInputs, const int numStates,
                               const size_t numTimeSamples,
                               const int trajectoryTimeLowerBound,
                               const int trajectoryTimeUpperBound);
  // TODO(tri-lucy) add options params
 private:
  const int numInputs_;
  const int numStates_;
  const size_t N;  // Number of time samples

  OptimizationProblem optProblem_;
  DecisionVariableView h_vars_;
};

}  // solvers
}  // drake
