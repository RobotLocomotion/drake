#pragma once

#include <cstdint>

#include "drake/common/trajectories/trajectory.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/system_output.h"

namespace drake {
namespace systems {

/// A source block which generates the value of a Trajectory for a
/// given time.  The output is vector values, and may vary with the
/// time (as reflected in the context) at which the output is
/// evaluated.
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
class TrajectorySource : public LeafSystem<T> {
 public:
  /// @param trajectory Trajectory used by the system.  This reference
  /// is aliased, and must remain valid for the lifetime of the
  /// system.
  explicit TrajectorySource(const Trajectory& trajectory);

 private:
  // Outputs a signal using the time-varying trajectory specified in the
  // constructor.
  void DoCalcOutput(const Context<T>& context,
                    SystemOutput<T>* output) const override;

  const Trajectory& trajectory_;
};

}  // namespace systems
}  // namespace drake
