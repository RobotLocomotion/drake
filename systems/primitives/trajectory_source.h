#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/trajectories/trajectory.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/single_output_vector_source.h"

namespace drake {
namespace systems {

/// Given a Trajectory, this System provides an output port with the value of
/// the trajectory evaluated at the current time.
///
/// If the particular Trajectory is not available at the time the System /
/// Diagram is being constructed, one can create a TrajectorySource with a
/// placeholder trajectory (e.g. PiecewisePolynomimal(Eigen::VectorXd)) with the
/// correct number of rows, and then use UpdateTrajectory().
///
/// @system
/// name: TrajectorySource
/// output_ports:
/// - y0
/// @endsystem
///
/// @tparam_double_only
/// @ingroup primitive_systems
template <typename T>
class TrajectorySource final : public SingleOutputVectorSource<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TrajectorySource)

  /// @param trajectory Trajectory used by the system.
  /// @param output_derivative_order The number of times to take the derivative.
  /// Must be greater than or equal to zero.
  /// @param zero_derivatives_beyond_limits All derivatives will be zero before
  /// the start time or after the end time of @p trajectory.
  /// @pre The value of `trajectory` is a column vector. More precisely,
  /// trajectory.cols() == 1.
  explicit TrajectorySource(const trajectories::Trajectory<T>& trajectory,
                            int output_derivative_order = 0,
                            bool zero_derivatives_beyond_limits = true);

  ~TrajectorySource() final = default;

  /// Updates the stored trajectory. @p trajectory must have the same number of
  /// rows as the trajectory passed to the constructor.
  void UpdateTrajectory(const trajectories::Trajectory<T>& trajectory);

 private:
  // Outputs a vector of values evaluated at the context time of the trajectory
  // and up to its Nth derivatives, where the trajectory and N are passed to
  // the constructor. The size of the vector is:
  // (1 + output_derivative_order) * rows of the trajectory passed to the
  // constructor.
  void DoCalcVectorOutput(
      const Context<T>& context,
      Eigen::VectorBlock<VectorX<T>>* output) const final;

  std::unique_ptr<trajectories::Trajectory<T>> trajectory_;
  const bool clamp_derivatives_;
  std::vector<std::unique_ptr<trajectories::Trajectory<T>>> derivatives_;
};

}  // namespace systems
}  // namespace drake
