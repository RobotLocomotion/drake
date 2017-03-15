#pragma once

#include "drake/systems/primitives/linear_system.h"

namespace drake {
namespace automotive {

/// SimplePowertrain models a powertrain with first-order lag. It accepts
/// throttle as the input and outputs the applied lumped force from the vehicle
/// to the road.
///
/// Input:
///  - A unitless scalar value representing the throttle input to the power
///    system.
///
/// Output:
///  - The force transmitted from the vehicle to the road [N].
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - AutoDiffXd
///
/// They are already available to link against in the containing library.
///
/// @ingroup automotive_systems
template <typename T>
class SimplePowertrain : public systems::LinearSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SimplePowertrain);

  /// Constructs a simple powertrain model, specified via a fixed time-constant
  /// and scalar gain.  The inputs are as follows:
  /// @param time_constant is the rise time of the first-order lag [s].
  /// @param gain is the gain converting throttle input to force output [N].
  SimplePowertrain(const double time_constant, const double gain)
      : systems::LinearSystem<T>(Vector1d(-1. / time_constant),
                                 Vector1d(gain),
                                 Vector1d(1. / time_constant),
                                 Vector1d(0.)),
        time_constant_(time_constant),
        gain_(gain) {}
  ~SimplePowertrain() override = default;

  const systems::InputPortDescriptor<T>& get_throttle_input_port() const {
    return systems::System<T>::get_input_port(0);
  }

  const systems::OutputPortDescriptor<T>& get_force_output_port() const {
    return systems::System<T>::get_output_port(0);
  }

  /// Accessors for the system constants.
  /// @{
  double get_time_constant() const { return time_constant_; }
  double get_gain() const { return gain_; }
  /// @}

 private:
  const double time_constant_{};
  const double gain_{};
};

}  // namespace automotive
}  // namespace drake
