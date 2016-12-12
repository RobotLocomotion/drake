#pragma once

#include <memory>

#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

/// A pass through system with input `u` and output `y = u`. This is
/// mathematically equivalent to a Gain system with its gain equal to one.
/// However this system incurs no computational cost.
/// The input to this system directly feeds through to its output.
/// This system is used, for instance, in PidController which is a Diagram
/// composed of simple framework primitives. In this case a PassThrough is used
/// to connect the exported input of the Diagram to the inputs of the Gain
/// systems for the proportioanal and integral constants of the controller. This
/// is necessary to provide an output port to which the internal Gain subsystems
/// connect. In this case the PassThrough is effectively creating an output port
/// that feeds through the input to the Diagram and that can now be connected to
/// the inputs of the inner subsystems to the Diagram.
/// A detailed discussion of the PidController can be found at
/// https://github.com/RobotLocomotion/drake/pull/3132.
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
///
/// This class uses Drake's `-inl.h` pattern.  When seeing linker errors from
/// this class, please refer to http://drake.mit.edu/cxx_inl.html.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - AutoDiffXd
///
/// They are already available to link against in libdrakeSystemFramework.
/// @ingroup primitive_systems

template <typename T>
class PassThrough : public LeafSystem<T> {
 public:
  /// Constructs a pass thorough system (`y = u`).
  /// @param size number of elements in the signal to be processed.
  explicit PassThrough(int size);

  /// Sets the output port to equal the input port.
  void EvalOutput(const Context<T>& context,
                  SystemOutput<T>* output) const override;
};

}  // namespace systems
}  // namespace drake
