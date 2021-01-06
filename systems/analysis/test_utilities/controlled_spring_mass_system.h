#pragma once

#include <memory>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/systems/analysis/test_utilities/spring_mass_system.h"
#include "drake/systems/controllers/pid_controller.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/primitives/constant_vector_source.h"

namespace drake {
namespace systems {

/// A model of a one-dimensional spring-mass system controlled to achieve a
/// given target position using a PID controller.
/// @see SpringMassSystem, PidController.
///
/// @tparam_nonsymbolic_scalar
/// @ingroup rigid_body_systems
template <typename T>
class PidControlledSpringMassSystem : public Diagram<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PidControlledSpringMassSystem)

  /// Constructs a spring-mass system with a fixed spring constant and given
  /// mass controlled by a PID controller to achieve a specified target
  /// position.
  /// @param[in] spring_stiffness The spring constant.
  /// @param[in] mass The value of the mass attached to the spring.
  /// @param[in] Kp the proportional constant.
  /// @param[in] Ki the integral constant.
  /// @param[in] Kd the derivative constant.
  /// @param[in] target_position the desired target position.
  PidControlledSpringMassSystem(double spring_stiffness, double mass,
                                double Kp, double Ki, double Kd,
                                const T& target_position);

  ~PidControlledSpringMassSystem() override {}

  T get_position(const Context<T>& context) const;

  T get_velocity(const Context<T>& context) const;

  T get_conservative_work(const Context<T>& context) const;

  /// Sets the position of the mass in the given Context.
  void set_position(Context<T>* context, const T& position) const;

  /// Sets the velocity of the mass in the given Context.
  void set_velocity(Context<T>* context, const T& position) const;

  /// Returns the SpringMassSystem plant of the model.
  const SpringMassSystem<T>& get_plant() const;

 private:
  // These are references into the Diagram; no ownership implied.
  SpringMassSystem<T>* plant_;
  controllers::PidController<T>* controller_;
  ConstantVectorSource<T>* target_;
};

}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class drake::systems::PidControlledSpringMassSystem)
