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

/// A source block that generates the value of a Trajectory for a given time.
/// The output is vector values, and may vary with the time (as reflected in
/// the context) at which the output is evaluated.
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
///
/// They are already available to link against in the containing library.
/// No other values for T are currently supported.
/// @ingroup primitive_systems
template <typename T>
class TrajectorySource : public SingleOutputVectorSource<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TrajectorySource)

  /// @param trajectory Trajectory used by the system.
  /// @param output_derivative_order The number of times to take the derivative.
  /// Must be greater than or equal to zero.
  /// @param zero_derivatives_beyond_limits All derivatives will be zero before
  /// the start time or after the end time of @p trajectory.
  explicit TrajectorySource(const Trajectory& trajectory,
                            int output_derivative_order = 0,
                            bool zero_derivatives_beyond_limits = true);

  ~TrajectorySource() override = default;

 protected:
  /// Outputs a vector of values evaluated at the context time of the trajectory
  /// and up to its Nth derivatives, where the trajectory and N are passed to
  /// the constructor. The size of the vector is:
  /// (1 + output_derivative_order) * rows of the trajectory passed to the
  /// constructor.
  void DoCalcVectorOutput(
      const Context<T>& context,
      Eigen::VectorBlock<VectorX<T>>* output) const override;

 private:
  const std::unique_ptr<Trajectory> trajectory_;
  const bool clamp_derivatives_;
  std::vector<std::unique_ptr<Trajectory>> derivatives_;
};

}  // namespace systems
}  // namespace drake
