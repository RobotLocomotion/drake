#pragma once

#include <memory>

#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

/// A pass through system with input `u` and output `y = u`. This is
/// mathematically equivalent to a Gain system with its gain equal to one.
/// However this system incurs no computational cost.
/// The input to this system directly feeds through to its output.
/// A detailed usage discussion of this system for a PID controller can be found
/// at https://github.com/RobotLocomotion/drake/pull/3132.
/// @tparam T The vector element type, which must be a valid Eigen scalar.
// TODO(amcastro-tri): cross reference PidController when implemented.
template <typename T>
class PassThrough : public LeafSystem<T> {
 public:
  /// Constructs a pass thorough system (`y = u`) with input/output ports of
  /// size @p. length.
  /// @param length is the size of the signal to be processed.
  explicit PassThrough(int length);

  /// Sets the output port to equal the input port.
  /// If the number of connected input or output ports is not one or the
  /// input ports are not of size length_, `std::runtime_error` will be thrown.
  void EvalOutput(const ContextBase<T>& context,
                  SystemOutput<T>* output) const override;
};

}  // namespace systems
}  // namespace drake
