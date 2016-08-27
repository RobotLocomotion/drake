#pragma once

#include <memory>

#include "drake/systems/framework/examples/spring_mass_system.h"
#include "drake/systems/framework/primitives/pid_controller.h"

namespace drake {
namespace systems {

/// A PID controller system. Given an error signal `e` and its derivative `edot`
/// the output of this sytem is
/// \f[
///   y = k_p \varepsilon + k_i \int{\varepsilon~dt} + k_d \dot{\varepsilon}
/// \f]
/// When the proportional constant is non-zero the input to this system
/// directly feeds through to its output.
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
template <typename T>
class PidControlledSpringMassSystem : public Diagram<T> {
 public:
  /// Constructs a PID controller with proportional constant @p Kp,
  /// integral constant @p Ki and derivative constant @p Kd.
  /// Input/output ports are limited to have size @p length.
  /// @param Kp the proportional constant.
  /// @param Ki the integral constant.
  /// @param Kd the derivative constant.
  /// @param length is the size of the signal to be processed.
  PidControlledSpringMassSystem(const T& Kp, const T& Ki, const T& Kd, int length);

  ~PidControlledSpringMassSystem() override {}

  // System<T> overrides
  bool has_any_direct_feedthrough() const override;

#if 0
  /// Sets @p context to a default state in which the integral of the
  /// controller is zero.
  void SetDefaultState(ContextBase<T>* context) const;

  /// Sets the integral of the %PidControlledSpringMassSystem to zero.
  /// @p value must be a column vector of the appropriate size.
  void set_integral_value(ContextBase<T>* context,
                          const Eigen::Ref<const VectorX<T>>& value) const;

  /// Returns the input port to the error signal.
  const SystemPortDescriptor<T>& get_error_signal_port() const;

  /// Returns the input port to the time derivative or rate of the error signal.
  const SystemPortDescriptor<T>& get_error_signal_rate_port() const;
#endif

 private:
  std::unique_ptr<SpringMassSystem> plant_;
  std::unique_ptr<PidController<T>> controller_;
};

}  // namespace systems
}  // namespace drake
