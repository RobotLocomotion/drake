#pragma once
#include <limits>
#include "drake/systems/vector.h"

namespace drake {
namespace systems {
template <template <typename> class Vector>
/**
 * Stores the sample time and values of the trajectory.
 * TODO(Hongkai) replace this with the "Trajectory" class to be implemented in
 * the future.
 */
struct TimeSampleTrajectory {
  std::vector<double> time;
  std::vector<Vector<double>, Eigen::aligned_allocator<Vector<double>>> val;
  TimeSampleTrajectory() : time(0), val(0) {}
};

/**
 * Implements a Drake System (see: drake/system/System.h) that saves the
 * entire simulated output in memory. This is useful for testing and
 * debugging the simulation results.
 */
template <template <typename> class Vector>
class TrajectoryLogger {
 public:
  explicit TrajectoryLogger(int traj_dim)
      : traj_dim_(traj_dim), cached_time(nullptr) {}
  ~TrajectoryLogger() { delete cached_time; }

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
    if (!cached_time ||
        std::abs(*cached_time - t) > std::numeric_limits<double>::epsilon()) {
      if (!cached_time) {
        cached_time = new double(t);
      } else {
        *cached_time = t;
      }
      trajectory_.time.push_back(t);
      trajectory_.val.push_back(u);
    }
    return u;
  }
  bool isTimeVarying() const { return false; }
  bool isDirectFeedthrough() const { return true; }
  size_t getNumStates() const { return 0u; }
  size_t getNumInputs() const { return traj_dim_; }
  size_t getNumOutputs() const { return traj_dim_; }

  TimeSampleTrajectory<Vector> getTrajectory() const { return trajectory_; }

 private:
  size_t traj_dim_;
  TimeSampleTrajectory<Vector> trajectory_;
  double* cached_time;
};
}  // namespace systems
}  // namespace drake
