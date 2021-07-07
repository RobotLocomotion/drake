#pragma once

#include <sstream>
#include <stdexcept>

#include "drake/common/drake_copyable.h"
#include "drake/systems/controllers/state_feedback_controller_interface.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {
namespace controllers {

// TODO(siyuanfeng): Lift the assumption that q and v have the same dimension.
// TODO(siyuanfeng): Generalize "q_d - q", e.g. for rotation.

// N.B. Inheritance order must remain fixed for pydrake (#9243).
/**
 * Implements the PID controller. Given estimated state `x_in = (q_in, v_in)`,
 * the controlled state `x_c = (q_c, v_c)` is computed by `x_c = P_x * x_in`,
 * where `P_x` is a state projection matrix. The desired state
 * `x_d = (q_d, v_d)`, is in the same space as `x_c`. The output of this
 * controller is:
 * <pre>
 * y = P_y * (kp * (q_d - q_c) + kd * (v_d - v_c) + ki * integral(q_d - q_c)),
 * </pre>
 * where `P_y` is the output projection matrix.
 *
 * @system
 * name: PidController
 * input_ports:
 * - estimated_state
 * - desired_state
 * output_ports:
 * - control
 * @endsystem
 *
 * This system has one continuous state, which is the integral of position
 * error, two input ports: estimated state `x_in` and desired state `x_d`, and
 * one output port `y`. Note that this class assumes `|q_c| = |v_c|` and
 * `|q_d| = |v_d|`. However, `|q_c|` does not have to equal to `|q_d|`. One
 * typical use case for non-identity `P_x` and `P_y` is to select a subset of
 * state for feedback.
 *
 * @tparam_default_scalar
 * @ingroup control_systems
 */
template <typename T>
class PidController : public LeafSystem<T>,
                      public StateFeedbackControllerInterface<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PidController)

  /**
   * Constructs a PID controller. `P_x` and `P_y` are identity
   * matrices of proper sizes. The estimated and desired state inputs are
   * 2 * @p kp's size, and the control output has @p kp's size.
   * @param kp P gain.
   * @param ki I gain.
   * @param kd D gain.
   *
   * @throws std::exception if @p kp, @p ki and @p kd have different
   * dimensions.
   */
  PidController(const Eigen::VectorXd& kp, const Eigen::VectorXd& ki,
                const Eigen::VectorXd& kd);

  /**
   * Constructs a PID controller. Calls the full constructor, with the output
   * projection matrix `P_y` being the identity matrix.
   * @param state_projection The state projection matrix `P_x`.
   * @param kp P gain.
   * @param ki I gain.
   * @param kd D gain.
   *
   * @throws std::exception if @p kp, @p ki and @p kd have different
   * dimensions or `P_x.row() != 2 * |kp|'.
   */
  PidController(const MatrixX<double>& state_projection,
                const Eigen::VectorXd& kp, const Eigen::VectorXd& ki,
                const Eigen::VectorXd& kd);

  /**
   * Constructs a PID controller. This assumes that
   * <pre>
   *   1. |kp| = |kd| = |ki| = |q_d| = |v_d|
   *   2. 2 * |q_d| = P_x.rows
   *   3. |x_in| = P_x.cols
   *   4. |y| = P_y.rows
   *   4. |q_d| = P_y.cols
   * </pre>
   *
   * @param state_projection The state projection matrix `P_x`.
   * @param output_projection The output projection matrix `P_y`.
   * @param kp P gain.
   * @param ki I gain.
   * @param kd V gain.
   *
   * @throws std::exception if any assumption is violated.
   */
  PidController(const MatrixX<double>& state_projection,
                const MatrixX<double>& output_projection,
                const Eigen::VectorXd& kp, const Eigen::VectorXd& ki,
                const Eigen::VectorXd& kd);

  /** Scalar-converting copy constructor.  See @ref system_scalar_conversion. */
  template <typename U>
  explicit PidController(const PidController<U>&);

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
    VectorBase<T>& state_vector =
        context->get_mutable_continuous_state_vector();
    state_vector.SetFromVector(value);
  }

  /**
   * Returns the input port for the estimated state.
   */
  const InputPort<T>& get_input_port_estimated_state() const final {
    return this->get_input_port(input_index_state_);
  }

  /**
   * Returns the input port for the desired state.
   */
  const InputPort<T>& get_input_port_desired_state() const final {
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
  void GetGraphvizFragment(int max_depth,
                           std::stringstream* dot) const override;

  void DoCalcTimeDerivatives(const Context<T>& context,
                             ContinuousState<T>* derivatives) const override;

 private:
  template <typename>
  friend class PidController;

  static double get_single_gain(const VectorX<double>& gain) {
    if (!gain.isConstant(gain[0])) {
      throw std::runtime_error("Gain is not singleton.");
    }
    return gain[0];
  }

  void CalcControl(const Context<T>& context, BasicVector<T>* control) const;

  VectorX<double> kp_;
  VectorX<double> ki_;
  VectorX<double> kd_;

  // Size of controlled positions / output.
  const int num_controlled_q_{0};
  // Size of input actual state.
  const int num_full_state_{0};
  // Projection matrix from full state to controlled state, whose size is
  // num_controlled_q_ * 2 X num_full_state_.
  const MatrixX<double> state_projection_;
  // Output projection matrix, whose size is num_controlled_q_ by the dimension
  // of the output port.
  const MatrixX<double> output_projection_;

  int input_index_state_{-1};
  int input_index_desired_state_{-1};
  int output_index_control_{-1};
};

}  // namespace controllers
}  // namespace systems
}  // namespace drake
