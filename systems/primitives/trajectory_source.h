#pragma once

#include <memory>
#include <utility>
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
/// Note: Scalar conversion is supported from double to any other scalar, but
/// the stored Trajectory is not automatically scalar converted. You must call
/// UpdateTrajectory() with an updated Trajectory<T> in order to fully enable
/// scalar-type support on the trajectory parameters/values.
///
/// @tparam_default_scalar @ingroup primitive_systems
template <typename T>
class TrajectorySource final : public SingleOutputVectorSource<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TrajectorySource);

  /// @param trajectory Trajectory used by the system.
  /// @param output_derivative_order The number of times to take the derivative.
  /// Must be greater than or equal to zero.
  /// @param zero_derivatives_beyond_limits All derivatives will be zero before
  /// the start time or after the end time of @p trajectory. However, this
  /// clamping is ignored for T=Expression.
  /// @pre The value of `trajectory` is a column vector. More precisely,
  /// trajectory.cols() == 1.
  explicit TrajectorySource(const trajectories::Trajectory<T>& trajectory,
                            int output_derivative_order = 0,
                            bool zero_derivatives_beyond_limits = true);

  // Scalar-converting copy constructor.  See @ref system_scalar_conversion.
  template <typename U>
  explicit TrajectorySource(const TrajectorySource<U>& other);

  ~TrajectorySource() final;

  /// Updates the stored trajectory. @p trajectory must have the same number of
  /// rows as the trajectory passed to the constructor.
  void UpdateTrajectory(const trajectories::Trajectory<T>& trajectory);

 private:
  // TrajectorySource of one scalar type is friends with all other scalar types.
  template <typename>
  friend class TrajectorySource;

  void CheckInvariants() const;

  // Outputs a vector of values evaluated at the context time of the trajectory
  // and up to its Nth derivatives, where the trajectory and N are passed to
  // the constructor. The size of the vector is:
  // (1 + output_derivative_order) * rows of the trajectory passed to the
  // constructor.
  void DoCalcVectorOutput(const Context<T>& context,
                          Eigen::VectorBlock<VectorX<T>>* output) const final;

  std::unique_ptr<trajectories::Trajectory<T>> trajectory_{};
  const bool clamp_derivatives_;
  std::vector<std::unique_ptr<trajectories::Trajectory<T>>> derivatives_{};

  // These are used when trajectory_ and derivatives_ are nullptr, e.g.,
  // after a scalar conversion. Scalar converting double => non-double ends
  // up cloning the T=double trajectory here, until UpdateTrajectory is used
  // to replace it.
  // TODO(russt): Remove these if/when we support automatic scalar conversion
  // of Trajectory classes.
  std::unique_ptr<trajectories::Trajectory<double>> failsafe_trajectory_{};
  std::vector<std::unique_ptr<trajectories::Trajectory<double>>>
      failsafe_derivatives_{};
};

namespace scalar_conversion {
/// Spells out the supported scalar conversions for TrajectorySource.
template <>
struct Traits<TrajectorySource> {
  template <typename T, typename U>
  using supported = typename std::bool_constant<
      // double -> anything
      (std::is_same_v<U, double>) ||
      // AutoDiffXd -> double (only when used for Clone())
      (std::is_same_v<std::pair<U, T>, std::pair<AutoDiffXd, double>>)>;
};
}  // namespace scalar_conversion

}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::TrajectorySource);
