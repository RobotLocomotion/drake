#pragma once

#include <memory>

#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/examples/spring_mass_system.h"
#include "drake/systems/framework/primitives/demultiplexer.h"
#include "drake/systems/framework/primitives/gain.h"
#include "drake/systems/framework/primitives/pid_controller2.h"

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
  PidControlledSpringMassSystem(const T& spring_stiffness, const T& mass,
                                const T& Kp, const T& Ki, const T& Kd);

  ~PidControlledSpringMassSystem() override {}

  void set_position(ContextBase<T>* context, const T& position) const;

  void set_velocity(ContextBase<T>* context, const T& position) const;

  const SpringMassSystem& get_plant() const { return *plant_.get(); }

  // System<T> overrides
  bool has_any_direct_feedthrough() const override;

  /// Sets @p context to a default state in which the integral of the
  /// controller is zero.
  void SetDefaultState(ContextBase<T>* context) const;

 private:
  std::unique_ptr<SpringMassSystem> plant_;
  std::unique_ptr<PidController<T>> controller_;
  std::unique_ptr<Demultiplexer<T>> demux_;
  std::unique_ptr<Gain<T>> inverter_;
};

}  // namespace systems
}  // namespace drake
