#pragma once

#include <memory>

#include "drake/automotive/bicycle.h"
#include "drake/automotive/simple_powertrain.h"
#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/primitives/linear_system.h"

namespace drake {
namespace automotive {

/// PoweredBicycle -- Model of a bicycle driven by a simplistic powertrain
/// element.  Specifically, PoweredBicycle is a diagram consisting of a linear
/// powertrain element connected in series with a three-DOF rigid-body bicycle
/// model (see Bicycle for details).  The diagram is assembled as follows:
///
/// <pre>
///   steering                                   +-------------+
///   (0) -------------------------------------->|             |      states
///               +------------+  body force     |   Bicycle   |-------> (0)
///   (1) ------->| Powertrain |---------------->|             |
///   throttle    +------------+                 +-------------+
/// </pre>
///
/// Inputs:
///  - Angle of the front wheel of the bicycle δ [rad]
///    (InputPortDescriptor getter: get_steering_input_port())
///  - A unitless scalar value representing the throttle input to the power
///    system.
///    (InputPortDescriptor getter: get_throttle_input_port())
///
/// Output:
///  - A scalar value representing the force input to the bicycle [N] (see
///    SimplePowertrain for details).
///    (OutputPortDescriptor getter: get_powertrain_output_port())
///  - A 6-dimensional vector containing the states of the bicycle (see
///    Bicycle for details).
///    (OutputPortDescriptor getter: get_state_output_port())
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - AutoDiffXd
/// - drake::symbolic::Expression
///
/// They are already available to link against in the containing library.
///
/// @ingroup automotive_systems
template <typename T>
class PoweredBicycle : public systems::Diagram<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PoweredBicycle)

  /// Constructor for the diagram.
  PoweredBicycle();

  ~PoweredBicycle() override = default;

  /// Accessors for the input and output ports.
  /// @{
  const systems::InputPortDescriptor<T>& get_steering_input_port() const {
    return systems::System<T>::get_input_port(0);
  }

  const systems::InputPortDescriptor<T>& get_throttle_input_port() const {
    return systems::System<T>::get_input_port(1);
  }

  const systems::OutputPortDescriptor<T>& get_powertrain_output_port() const {
    return systems::System<T>::get_output_port(0);
  }

  const systems::OutputPortDescriptor<T>& get_state_output_port() const {
    return systems::System<T>::get_output_port(1);
  }
  /// @}

  /// Accessors for the Bicycle and SimplePowertrain systems.
  /// @{
  const Bicycle<T>* get_bicycle_system() const { return bike_; }
  const SimplePowertrain<T>* get_powertrain_system() const { return power_; }
  /// @}

 private:
  const Bicycle<T>* bike_{nullptr};
  const SimplePowertrain<T>* power_{nullptr};
};

}  // namespace automotive
}  // namespace drake
