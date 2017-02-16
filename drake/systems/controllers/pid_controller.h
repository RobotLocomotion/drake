#pragma once

#include <memory>
#include <utility>

#include "drake/common/drake_copyable.h"
#include "drake/systems/controllers/controller_base.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/primitives/adder.h"
#include "drake/systems/primitives/gain.h"
#include "drake/systems/primitives/integrator.h"
#include "drake/systems/primitives/pass_through.h"
#include "drake/systems/primitives/matrix_gain.h"

namespace drake {
namespace systems {

template <typename T>
class PidControllerInternal;

template <typename T>
class PidController : public Controller<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PidController)

  PidController(int state_dim, int control_dim,
      const VectorX<T>& kp, const VectorX<T>& ki, const VectorX<T>& kd) {
    ConnectPorts(nullptr, state_dim, control_dim, kp, ki, kd);
  }

  PidController(std::unique_ptr<MatrixGain<T>> feedback_selector,
      int state_dim, int control_dim,
      const VectorX<T>& kp, const VectorX<T>& ki, const VectorX<T>& kd) {
    ConnectPorts(std::move(feedback_selector),
                 state_dim, control_dim, kp, ki, kd);
  }

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

  /// Sets the integral of the %PidControllerInternal to @p value.
  /// @p value must be a column vector of the appropriate size.
  void set_integral_value(Context<T>* context,
                          const Eigen::Ref<const VectorX<T>>& value) const;

 private:
  void ConnectPorts(std::unique_ptr<MatrixGain<T>> feedback_selector,
                    int state_dim, int control_dim,
                    const VectorX<T>& kp, const VectorX<T>& ki,
                    const VectorX<T>& kd);

  PidControllerInternal<T>* controller_;
};

}  // namespace systems
}  // namespace drake
