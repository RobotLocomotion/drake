#pragma once

#include <memory>

#include "drake/automotive/bicycle.h"
#include "drake/automotive/simple_power_train.h"
#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/primitives/linear_system.h"

namespace drake {
namespace automotive {

/// Model of a powered bicycle, a system consisting of a linear power-train
/// element connected in series with rigid body bicycle equations-of-motion.
/// This is assembled according to the following diagram:
///
///  steering
///  (0) ------------------------------------+   +-------------+
///                                          +-->|             |      states
///  throttle    +-------------+  body force     |   Bicycle   |-------> (0)
///  (1) ------->| Power-train |---------------->|             |
///              +-------------+                 +-------------+
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

  /// Accessors for the inputs and output.
  const systems::InputPortDescriptor<T>& get_steering_input_port() const {
    return systems::System<T>::get_input_port(0);
  }

  const systems::InputPortDescriptor<T>& get_throttle_input_port() const {
    return systems::System<T>::get_input_port(1);
  }

  const systems::OutputPortDescriptor<T>& get_state_output_port() const {
    return systems::System<T>::get_output_port(0);
  }

  /// Accessors for the ego and agent car systems.
  const Bicycle<T>* get_bicycle_system() const { return bike_; }
  const SimplePowerTrain<T>* get_powertrain_system() const {
    return power_;
  }

 private:
  const Bicycle<T>* bike_ = nullptr;
  const SimplePowerTrain<T>* power_ = nullptr;
};

}  // namespace automotive
}  // namespace drake
