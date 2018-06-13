#pragma once

#include "drake/automotive/gen/driving_command.h"
#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace automotive {

/// A special-purpose multiplexer that packs two scalar inputs, steering angle
/// (in units rad) and acceleration (in units m/s^2), into a vector-valued
/// output of type DrivingCommand<T>, where the inputs feed directly through to
/// the output.
///
/// This class differs from systems::Multiplexer<T> constructed with a
/// DrivingCommand<T> model vector because a BasicVector (and notably any of its
/// subclasses) stored as T=double cannot yet be converted to another type.  See
/// #8921.
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
///
/// Instantiated templates for the following `T` values are provided:
/// - double
/// - AutoDiffXd
/// - symbolic::Expression
///
/// They are already available to link against in the containing library.
/// Currently, no other values for `T` are supported.
///
/// @ingroup automotive_systems
template <typename T>
class DrivingCommandMux : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DrivingCommandMux)

  /// Constructs a %DrivingCommandMux with two scalar-valued input ports, and
  /// one output port containing a DrivingCommand<T>.
  DrivingCommandMux();

  /// Scalar-converting copy constructor. See @ref system_scalar_conversion.
  template <typename U>
  explicit DrivingCommandMux(const DrivingCommandMux<U>&);

  /// See the class description for details on the following input ports.
  /// @{
  const systems::InputPortDescriptor<T>& steering_input() const;
  const systems::InputPortDescriptor<T>& acceleration_input() const;
  /// @}

 private:
  // Packs a DrivingCommand based on the values seen at the input ports.
  void CombineInputsToOutput(const systems::Context<T>& context,
                             DrivingCommand<T>* output) const;

  const int steering_port_index_{};
  const int acceleration_port_index_{};
};

}  // namespace automotive
}  // namespace drake
