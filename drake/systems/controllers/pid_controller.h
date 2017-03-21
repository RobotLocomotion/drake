#pragma once

#include <fstream>
#include <memory>
#include <utility>

#include "drake/common/drake_copyable.h"
#include "drake/systems/controllers/state_feedback_controller_base.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/primitives/matrix_gain.h"

namespace drake {
namespace systems {

// TODO(siyuanfeng): Need to redo the PID controller, then this would go away.
template <typename T>
class PidControllerInternal;

// TODO(siyuanfeng): Lift the assumption that q and v have the same dimension.
// TODO(siyuanfeng): Generalize "q_d - q", e.g. for rotation.

/**
 * Implements the PID controller. Given state `(q, v)`, desired state
 * `(q_d, v_d)`, the output of this controller is
 * <pre>
 * y = kp * (q_d - q) + kd * (v_d - v) + ki * integral(q_d - q, dt),
 * </pre>
 * where `integral(q_d - q, dt)` is the integrated position error.
 *
 * Note that this class assumes q and v have the same dimension.
 */
template <typename T>
class PidController : public StateFeedbackController<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PidController)

  /**
   * Constructs a PID controller. Assumes that @p kp, @p ki and @p kd have the
   * same size, the actual and desired state inputs will have the size of
   * 2 * @p kp's size, and the control output will have @p kp's size.
   * @param kp P gain.
   * @param ki I gain.
   * @param kd D gain.
   */
  PidController(
      const VectorX<T>& kp, const VectorX<T>& ki, const VectorX<T>& kd);

  /**
   * Constructs a PID controller where some of the input states may not be
   * controlled. Assumes that @p kp, @p ki and @p kd have the same size. The
   * actual and desired state input's size and the control output's size need
   * to match @p feedback_selector.
   * @param feedback_selector, The selection matrix indicating controlled
   * states, whose size should be 2 * @p kp's size by the size of the full
   * state.
   * @param kp P gain.
   * @param ki I gain.
   * @param kd D gain.
   */
  PidController(std::unique_ptr<MatrixGain<T>> feedback_selector,
      const VectorX<T>& kp, const VectorX<T>& ki, const VectorX<T>& kd);

  /// Returns the proportional gain constant. This method should only be called
  /// if the proportional gain can be represented as a scalar value, i.e., every
  /// element in the proportional gain vector is the same. It will throw a
  /// `std::runtime_error` if the proportional gain cannot be represented as a
  /// scalar value.
  const T& get_Kp_singleton() const;

  /// Returns the integral gain constant. This method should only be called if
  /// the integral gain can be represented as a scalar value, i.e., every
  /// element in the integral gain vector is the same. It will throw a
  /// `std::runtime_error` if the integral gain cannot be represented as a
  /// scalar value.
  const T& get_Ki_singleton() const;

  /// Returns the derivative gain constant. This method should only be called if
  /// the derivative gain can be represented as a scalar value, i.e., every
  /// element in the derivative gain vector is the same. It will throw a
  /// `std::runtime_error` if the derivative gain cannot be represented as a
  /// scalar value.
  const T& get_Kd_singleton() const;

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

  /// Sets the integral part of the PidController to @p value.
  /// @p value must be a column vector of the appropriate size.
  void set_integral_value(Context<T>* context,
                          const Eigen::Ref<const VectorX<T>>& value) const;

 protected:
  /// Appends to @p dot a simplified Graphviz representation of the PID
  /// controller, since the internal wiring is unimportant and hard for human
  /// viewers to parse.
  void GetGraphvizFragment(std::stringstream* dot) const override;
  /// Appends the Graphviz port for @p port to @p dot.
  void GetGraphvizInputPortToken(const InputPortDescriptor<T>& port,
                                 std::stringstream* dot) const override;
  /// Appends the Graphviz port for @p port to @p dot.
  void GetGraphvizOutputPortToken(const OutputPortDescriptor<T>& port,
                                  std::stringstream* dot) const override;

 private:
  void ConnectPorts(std::unique_ptr<MatrixGain<T>> feedback_selector,
                    const VectorX<T>& kp, const VectorX<T>& ki,
                    const VectorX<T>& kd);

  // TODO(siyuanfeng): Need to redo the PID controller, then this would go away.
  PidControllerInternal<T>* controller_;
};

}  // namespace systems
}  // namespace drake
