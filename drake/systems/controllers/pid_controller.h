#pragma once

#include <fstream>
#include <memory>
#include <utility>

#include "drake/common/drake_copyable.h"
#include "drake/systems/controllers/state_feedback_controller_interface.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/primitives/matrix_gain.h"

namespace drake {
namespace systems {

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
 * This system has one continuous state which is the integral of position error,
 * two input ports: estimated state (q, v) and desired state (q_d, v_d), and
 * one output port y.
 *
 * Note that this class assumes |q| = |v|, and |q_d| = |v_d|. Also |q| >= |q_d|.
 * The user can specify a selection matrix that picks the *controlled* states
 * from (q, v) for feedback. See constructor documentations for more details.
 *
 * @tparam T The vector element type, which must be a valid Eigen scalar.
 *
 * Instantiated templates for the following kinds of T's are provided:
 * - double
 * - AutoDiffXd
 * - symbolic::Expression
 *
 * @ingroup control_systems
 */
template <typename T>
class PidController : public StateFeedbackControllerInterface<T>,
                      public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PidController)

  /**
   * Constructs a PID controller. Assumes that @p kp, @p ki and @p kd have the
   * same size, the estimated and desired state inputs will have the size of
   * 2 * @p kp's size, and the control output will have @p kp's size.
   * @param kp P gain.
   * @param ki I gain.
   * @param kd D gain.
   */
  PidController(const Eigen::VectorXd& kp, const Eigen::VectorXd& ki,
                const Eigen::VectorXd& kd);

  /**
   * Constructs a PID controller where some of the input states may not be
   * controlled. Assumes that @p kp, @p ki and @p kd have the same size. The
   * estimated and desired state input's size and the control output's size need
   * to match @p feedback_selector. Note that @p state_selector only affects
   * the estimated state input but not the desired state.
   * @param feedback_selector, The selection matrix indicating controlled
   * states, whose size should be 2 * @p kp's size by the size of the full
   * state.
   * @param kp P gain.
   * @param ki I gain.
   * @param kd D gain.
   */
  PidController(const MatrixX<double>& state_selector,
                const Eigen::VectorXd& kp, const Eigen::VectorXd& ki,
                const Eigen::VectorXd& kd);

  /**
   * Returns the proportional gain constant. This method should only be called
   * if the proportional gain can be represented as a scalar value, i.e., every
   * element in the proportional gain vector is the same. It will throw a
   * `std::runtime_error` if the proportional gain cannot be represented as a
   * scalar value.
   */
  double get_Kp_singleton() const { return get_single_gain(kp_); }

  /**
   * Returns the integral gain constant. This method should only be called if
   * the integral gain can be represented as a scalar value, i.e., every
   * element in the integral gain vector is the same. It will throw a
   * `std::runtime_error` if the integral gain cannot be represented as a
   * scalar value.
   */
  double get_Ki_singleton() const { return get_single_gain(ki_); }

  /**
   * Returns the derivative gain constant. This method should only be called if
   * the derivative gain can be represented as a scalar value, i.e., every
   * element in the derivative gain vector is the same. It will throw a
   * `std::runtime_error` if the derivative gain cannot be represented as a
   * scalar value.
   */
  double get_Kd_singleton() const { return get_single_gain(kd_); }

  /**
   * Returns the proportional gain vector.
   */
  const VectorX<double>& get_Kp_vector() const { return kp_; }

  /**
   * Returns the integral gain vector.
   */
  const VectorX<double>& get_Ki_vector() const { return ki_; }

  /**
   * Returns the derivative gain vector.
   */
  const VectorX<double>& get_Kd_vector() const { return kd_; }

  /**
   * Sets the integral part of the PidController to @p value.
   * @p value must be a column vector of the appropriate size.
   */
  void set_integral_value(Context<T>* context,
                          const Eigen::Ref<const VectorX<T>>& value) const {
    VectorBase<T>* state_vector =
        context->get_mutable_continuous_state_vector();
    state_vector->SetFromVector(value);
  }

  /**
   * Returns the input port for the estimated state.
   */
  const InputPortDescriptor<T>& get_input_port_estimated_state() const final {
    return this->get_input_port(input_index_state_);
  }

  /**
   * Returns the input port for the desired state.
   */
  const InputPortDescriptor<T>& get_input_port_desired_state() const final {
    return this->get_input_port(input_index_desired_state_);
  }

  /**
   * Returns the output port for computed control.
   */
  const OutputPort<T>& get_output_port_control() const final {
    return this->get_output_port(output_index_control_);
  }

 protected:
  /**
   * Appends to @p dot a simplified Graphviz representation of the PID
   * controller, since the internal wiring is unimportant and hard for human
   * viewers to parse.
   */
  void GetGraphvizFragment(std::stringstream* dot) const override;

  PidController<symbolic::Expression>* DoToSymbolic() const override;

  void DoCalcTimeDerivatives(const Context<T>& context,
                             ContinuousState<T>* derivatives) const override;

 private:
  static double get_single_gain(const VectorX<double>& gain) {
    if (!gain.isConstant(gain[0])) {
      throw std::runtime_error("Gain is not singleton.");
    }
    return gain[0];
  }

  void CalcControl(const Context<T>& context, BasicVector<T>* control) const;

  VectorX<double> kp_;
  VectorX<double> kd_;
  VectorX<double> ki_;

  // Size of controlled positions / output.
  const int num_controlled_q_{0};
  // Size of input actual state.
  const int num_full_state_{0};
  // Projection matrix from full state to controlled state, whose size is
  // num_controlled_q_ * 2 X num_full_state_.
  const MatrixX<double> state_selector_;

  int input_index_state_{-1};
  int input_index_desired_state_{-1};
  int output_index_control_{-1};
};

}  // namespace systems
}  // namespace drake
