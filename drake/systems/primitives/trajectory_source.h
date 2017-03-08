#pragma once

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

  /// @param trajectory Trajectory used by the system.  This reference is
  /// aliased, and must remain valid for the lifetime of the system.
  explicit TrajectorySource(const Trajectory& trajectory);

  ~TrajectorySource() override = default;

 protected:
  /// Outputs a signal using the time-varying trajectory specified in the
  /// constructor.
  void DoCalcVectorOutput(
      const Context<T>& context,
      Eigen::VectorBlock<VectorX<T>>* output) const override;

 private:
  const Trajectory& trajectory_;
};

}  // namespace systems
}  // namespace drake
