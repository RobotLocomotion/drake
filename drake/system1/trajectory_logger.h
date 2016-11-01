#pragma once

#include <cmath>

#include "drake/system1/vector.h"

namespace drake {
namespace systems {
template <template <typename> class Vector>
/**
 * Stores the sample times and values of the trajectory.
 * TODO(Hongkai) replace this with the "Trajectory" class to be implemented in
 * the future.
 */
struct TimeSampleTrajectory {
  std::vector<double> time;
  std::vector<Vector<double>, Eigen::aligned_allocator<Vector<double>>> value;
  TimeSampleTrajectory() : time(0), value(0) {}
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * Implements a Drake System (see: drake/system/System.h) that saves the
 * entire simulated output in memory. This is useful for testing and
 * debugging the simulation results.
 */
template <template <typename> class Vector>
class TrajectoryLogger {
 public:
  explicit TrajectoryLogger(int traj_dim) : traj_dim_(traj_dim) {}

  // Noncopyable
  TrajectoryLogger(const TrajectoryLogger&) = delete;
  TrajectoryLogger& operator=(const TrajectoryLogger&) = delete;
  TrajectoryLogger(TrajectoryLogger&&) = delete;
  TrajectoryLogger& operator=(TrajectoryLogger&&) = delete;

  template <typename ScalarType>
  using StateVector = NullVector<ScalarType>;
  template <typename ScalarType>
  using InputVector = Vector<ScalarType>;
  template <typename ScalarType>
  using OutputVector = Vector<ScalarType>;

  template <typename ScalarType>
  StateVector<ScalarType> dynamics(const double& t,
                                   const StateVector<ScalarType>& x,
                                   const InputVector<ScalarType>& u) const {
    return StateVector<ScalarType>();
  }

  template <typename ScalarType>
  OutputVector<ScalarType> output(const double& t,
                                  const StateVector<ScalarType>& x,
                                  const InputVector<ScalarType>& u) {
    if (trajectory_.time.empty() ||
        trajectory_.time[trajectory_.time.size() - 1] != t) {
      trajectory_.time.push_back(t);
      trajectory_.value.push_back(u);
    }
    return u;
  }
  bool isTimeVarying() const { return false; }
  bool isDirectFeedthrough() const { return true; }
  size_t getNumStates() const { return 0u; }
  size_t getNumInputs() const { return traj_dim_; }
  size_t getNumOutputs() const { return traj_dim_; }

  TimeSampleTrajectory<Vector> getTrajectory() const { return trajectory_; }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  size_t traj_dim_;
  TimeSampleTrajectory<Vector> trajectory_;
};
}  // namespace systems
}  // namespace drake
