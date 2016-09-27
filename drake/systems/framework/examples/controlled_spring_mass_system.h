#pragma once

#include <memory>

#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/examples/spring_mass_system.h"
#include "drake/systems/framework/primitives/adder.h"
#include "drake/systems/framework/primitives/constant_vector_source.h"
#include "drake/systems/framework/primitives/demultiplexer.h"
#include "drake/systems/framework/primitives/gain.h"
#include "drake/systems/framework/primitives/pid_controller.h"

namespace drake {
namespace systems {

/// A model of a one-dimensional spring-mass system controlled to achieve a
/// given target position using a PID controller.
/// @see SpringMassSystem, PidController.
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - AutoDiffXd
///
/// They are already available to link against in libdrakeSystemFramework.
/// No other values for T are currently supported.
/// @ingroup systems
template <typename T>
class PidControlledSpringMassSystem : public Diagram<T> {
 public:
  /// Constructs a spring-mass system with a fixed spring constant and given
  /// mass controlled by a PID controller to achieve a specified target
  /// position.
  /// @param[in] spring_stiffness The spring constant.
  /// @param[in] mass The value of the mass attached to the spring.
  /// @param[in] Kp the proportional constant.
  /// @param[in] Ki the integral constant.
  /// @param[in] Kd the derivative constant.
  /// @param[in] target_position the desired target position.
  PidControlledSpringMassSystem(const T& spring_stiffness, const T& mass,
                                const T& Kp, const T& Ki, const T& Kd,
                                const T& target_position);

  ~PidControlledSpringMassSystem() override {}

  /// Sets the position of the mass in the given Context.
  void set_position(Context<T>* context, const T& position) const;

  /// Sets the velocity of the mass in the given Context.
  void set_velocity(Context<T>* context, const T& position) const;

  /// Returns the SpringMassSystem plant of the model.
  const SpringMassSystem<T>& get_plant() const;

  // System<T> overrides
  bool has_any_direct_feedthrough() const override;

  /// Sets @p context to a default state in which the position and velocity of
  /// the mass are both zero.
  /// The integral of the controller is also set to zero.
  void SetDefaultState(Context<T>* context) const;

 private:
  // These are references into the Diagram; no ownership implied.
  SpringMassSystem<T>* plant_;
  PidController<T>* controller_;
  Demultiplexer<T>* demux_;
  Gain<T>* pid_inverter_;
  Gain<T>* target_inverter_;
  ConstantVectorSource<T>* target_;
  Adder<T>* state_minus_target_;
};

}  // namespace systems
}  // namespace drake
