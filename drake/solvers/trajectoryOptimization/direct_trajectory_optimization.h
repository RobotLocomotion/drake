#pragma once

#include <cstddef>

#include <Eigen/Core>

#include "drake/drakeOptimization_export.h"
#include "drake/solvers/Optimization.h"

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
  const size_t N; // Number of time samples

  OptimizationProblem optProblem_;
  // TODO(tri-lucy): I'm not sure we actually need these inds, because of how
  // vars are made in C++ Drake.
  Eigen::VectorXd h_inds_;  // (N-1) x 1 indices for timesteps h so that
                            // z(h_inds(i)) = h(i)
  Eigen::MatrixXd x_inds_;  // n x N indices for state
  Eigen::MatrixXd u_inds_;  // m x N indices for time
};
}  // solvers
}  // drake
