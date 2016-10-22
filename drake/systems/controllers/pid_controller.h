#pragma once

#include <memory>

#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/primitives/adder.h"
#include "drake/systems/framework/primitives/gain.h"
#include "drake/systems/framework/primitives/integrator.h"
#include "drake/systems/framework/primitives/pass_through.h"

namespace drake {
namespace systems {

/// A PID controller system. Given an error signal `e` and its time derivative
/// `edot` the output of this sytem is
/// <pre>
///     y = Kp * e + Ki integ(e,dt) + Kd * edot
/// </pre>
/// where `integ(e,dt)` is the time integral of `e`.
/// A PID controller directly feedthroughs the error signal to the output when
/// the proportional constant is non-zero. It feeds through the rate of change
/// of the error signal when the derivative constant is non-zero.
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - AutoDiffXd
///
/// They are already available to link against in libdrakeSystemFramework.
/// No other values for T are currently supported.
/// @ingroup primitive_systems
template <typename T>
class PidController : public Diagram<T> {
 public:
  /// Constructs a %PidController system where all of the gains are the same
  /// value.
  ///
  /// @param Kp the proportional constant.
  /// @param Ki the integral constant.
  /// @param Kd the derivative constant.
  /// @param size number of elements in the signal to be processed.
  PidController(const T& Kp, const T& Ki, const T& Kd, int size);

  /// Constructs a %PidController system where each gain can have a different
  /// value.
  ///
  /// @param Kp the vector of proportional gain constants.
  /// @param Ki the vector of integral gain constants.
  /// @param Kd the vector of derivative gain constants.
  PidController(const VectorX<T>& Kp, const VectorX<T>& Ki,
                const VectorX<T>& Kd);

  ~PidController() override {}

  /// Returns the proportional gain constant. This method should only be called
  /// if the proportional gain can be represented as a scalar value, i.e., every
  /// element in the proportional gain vector is the same. It will throw a
  /// `std::runtime_error` if the proportional gain cannot be represented as a
  /// scalar value.
  const T& get_Kp() const;

  /// Returns the integral gain constant. This method should only be called if
  /// the integral gain can be represented as a scalar value, i.e., every
  /// element in the integral gain vector is the same. It will throw a
  /// `std::runtime_error` if the integral gain cannot be represented as a
  /// scalar value.
  const T& get_Ki() const;

  /// Returns the derivative gain constant. This method should only be called if
  /// the derivative gain can be represented as a scalar value, i.e., every
  /// element in the derivative gain vector is the same. It will throw a
  /// `std::runtime_error` if the derivative gain cannot be represented as a
  /// scalar value.
  const T& get_Kd() const;

  /// Returns the proportional vector constant.
  const VectorX<T>& get_Kp_vector() const;

  /// Returns the integral vector constant.
  const VectorX<T>& get_Ki_vector() const;

  /// Returns the derivative vector constant.
  const VectorX<T>& get_Kd_vector() const;

  // System<T> overrides
  /// A PID controller directly feedthroughs the error signal to the output when
  /// the proportional constant is non-zero. It feeds through the rate of change
  /// of the error signal when the derivative constant is non-zero.
  bool has_any_direct_feedthrough() const override;

  /// Sets @p context to a default state in which the integral of the error
  /// signal is zero.
  void SetDefaultState(Context<T>* context) const;

  /// Sets the integral of the %PidController to @p value.
  /// @p value must be a column vector of the appropriate size.
  void set_integral_value(Context<T>* context,
                          const Eigen::Ref<const VectorX<T>>& value) const;

  /// Returns the input port to the error signal.
  const SystemPortDescriptor<T>& get_error_port() const;

  /// Returns the input port to the time derivative or rate of the error signal.
  const SystemPortDescriptor<T>& get_error_derivative_port() const;

  /// Returns the output port to the control output.
  const SystemPortDescriptor<T>& get_control_output_port() const;

 private:
  Adder<T>* adder_ = nullptr;
  Integrator<T>* integrator_ = nullptr;
  PassThrough<T>* pass_through_ = nullptr;
  Gain<T>* proportional_gain_ = nullptr;
  Gain<T>* integral_gain_ = nullptr;
  Gain<T>* derivative_gain_ = nullptr;
};

}  // namespace systems
}  // namespace drake
