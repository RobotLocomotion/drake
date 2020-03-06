#pragma once

#include <memory>
#include <utility>

#include "drake/common/drake_copyable.h"
#include "drake/systems/controllers/pid_controller.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/system.h"
#include "drake/systems/primitives/adder.h"
#include "drake/systems/primitives/constant_vector_source.h"

namespace drake {
namespace systems {
namespace controllers {

/// A system that encapsulates a PidController and a controlled System (a.k.a
/// the "plant").
///
/// The passed in plant must meet the following properties:
///
/// * Input port zero must be all of the control inputs (size U). When the plant
///   is a dynamics model, this is typically the generalized effort (e.g., force
///   or torque) command.
///
/// * The output port passed to the PidControlledSystem constructor must be
///   of size 2 * Q, where the first Q elements are the position states of
///   the plant, and the second Q elements are the velocity states of the
///   plant. Q >= U.
///
/// The resulting PidControlledSystem has two input ports with the following
/// properties:
///
/// * Input port zero is the feed forward control (size U), which will be added
///   onto the output of the PID controller. The sum is sent to the plant's
///   input.
///
/// * Input port one is the desired *controlled* states (2 * U) of the plant,
///   where the first half are the *controlled* positions, and the second half
///   are the *controlled* velocities.
///
/// All output ports of the plant are exposed as output ports of the
/// PidControlledSystem in the same order (and therefore with the same index)
/// as they appear in the plant.
///
/// Some of the constructors include a parameter called `feedback_selector`.
/// It is used to select the *controlled* states from the plant's state output
/// port. Let `S` be the gain matrix in parameter `feedback_selector`. `S` must
/// have dimensions of `(2 * U, 2 * Q)`. Typically, `S` contains one `1` in each
/// row, and zeros everywhere else. `S` does not affect the desired state input.
/// Let 'x' be the full state of the plant (size 2 * Q), and 'x_d' be the
/// desired state (size 2 * U), `S` is used to compute the state error as
/// `x_err = S * x - x_d`.
///
/// @tparam_nonsymbolic_scalar
/// @ingroup control_systems
template <typename T>
class PidControlledSystem : public Diagram<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PidControlledSystem)

  /// @p plant full state is used for feedback control, and all the dimensions
  /// have homogeneous gains specified by @p Kp, @p Kd and @p Ki.
  ///
  /// @param[in] plant The system to be controlled. This must not be `nullptr`.
  /// @param[in] Kp the proportional constant.
  /// @param[in] Ki the integral constant.
  /// @param[in] Kd the derivative constant.
  /// @param[in] state_output_port_index identifies the output port on the
  /// plant that contains the (full) state information.
  ///
  /// @pydrake_mkdoc_identifier{5args_double_gains}
  PidControlledSystem(std::unique_ptr<System<T>> plant, double Kp, double Ki,
                      double Kd, int state_output_port_index = 0);

  /// @p plant full state is used for feedback control, and the vectorized gains
  /// are specified by @p Kp, @p Kd and @p Ki.
  ///
  /// @param[in] plant The system to be controlled. This must not be `nullptr`.
  /// @param[in] Kp the proportional vector constant.
  /// @param[in] Ki the integral vector constant.
  /// @param[in] Kd the derivative vector constant.
  /// @param[in] state_output_port_index identifies the output port on the
  /// plant that contains the (full) state information.
  ///
  /// @pydrake_mkdoc_identifier{5args_vector_gains}
  PidControlledSystem(std::unique_ptr<System<T>> plant,
                      const Eigen::VectorXd& Kp, const Eigen::VectorXd& Ki,
                      const Eigen::VectorXd& Kd,
                      int state_output_port_index = 0);

  /// A constructor where the gains are scalar values and some of the plant's
  /// output is part of the feedback signal as specified by
  /// @p feedback_selector.
  ///
  /// @param[in] plant The system to be controlled. This must not be `nullptr`.
  /// @param[in] feedback_selector The matrix that selects which part of the
  /// plant's full state is fed back to the PID controller. For semantic details
  /// of this parameter, see this class's description.
  /// @param[in] Kp the proportional constant.
  /// @param[in] Ki the integral constant.
  /// @param[in] Kd the derivative constant.
  /// @param[in] state_output_port_index identifies the output port on the
  /// plant that contains the (full) state information.
  ///
  /// @pydrake_mkdoc_identifier{6args_double_gains}
  PidControlledSystem(std::unique_ptr<System<T>> plant,
                      const MatrixX<double>& feedback_selector, double Kp,
                      double Ki, double Kd, int state_output_port_index = 0);

  /// A constructor where the gains are vector values and some of the plant's
  /// output is part of the feedback signal as specified by
  /// @p feedback_selector.
  ///
  /// @param[in] plant The system to be controlled. This must not be `nullptr`.
  /// @param[in] feedback_selector The matrix that selects which part of the
  /// plant's full state is fed back to the PID controller. For semantic details
  /// of this parameter, see this class's description.
  /// @param[in] Kp the proportional vector constant.
  /// @param[in] Ki the integral vector constant.
  /// @param[in] Kd the derivative vector constant.
  /// @param[in] state_output_port_index identifies the output port on the
  /// plant that contains the (full) state information.
  ///
  /// @pydrake_mkdoc_identifier{6args_vector_gains}
  PidControlledSystem(std::unique_ptr<System<T>> plant,
                      const MatrixX<double>& feedback_selector,
                      const Eigen::VectorXd& Kp, const Eigen::VectorXd& Ki,
                      const Eigen::VectorXd& Kd,
                      int state_output_port_index = 0);

  ~PidControlledSystem() override;

  System<T>* plant() { return plant_; }

  /// @return the input port for the feed forward control input.
  const InputPort<T>& get_control_input_port() const {
    return this->get_input_port(0);
  }

  /// @return the input port for the desired position/velocity state.
  const InputPort<T>& get_state_input_port() const {
    return this->get_input_port(1);
  }

  const OutputPort<T>& get_state_output_port() const {
    return this->get_output_port(state_output_port_index_);
  }

  /// The return type of ConnectController.
  struct ConnectResult {
    /// The feed forward control input.
    const InputPort<T>& control_input_port;
    /// The feedback state input.
    const InputPort<T>& state_input_port;
  };

  /// Creates a PidController and uses @p builder to connect @p plant_input and
  /// @p plant_output from an existing plant. The controlled states are selected
  /// by @p feedback_selector.
  static ConnectResult ConnectController(
      const InputPort<T>& plant_input,
      const OutputPort<T>& plant_output,
      const MatrixX<double>& feedback_selector,
      const Eigen::VectorXd& Kp, const Eigen::VectorXd& Ki,
      const Eigen::VectorXd& Kd, DiagramBuilder<T>* builder);

  /// Creates a PidController and uses @p builder to connect @p plant_input and
  /// @p plant_output from an existing plant. The plant's full state is used for
  /// feedback.
  static ConnectResult ConnectController(
      const InputPort<T>& plant_input,
      const OutputPort<T>& plant_output,
      const Eigen::VectorXd& Kp, const Eigen::VectorXd& Ki,
      const Eigen::VectorXd& Kd, DiagramBuilder<T>* builder);

  /// Creates a PidController with input saturation and uses @p builder to
  /// connect @p plant_input and @p plant_output from an existing plant. The
  /// controlled states are selected by @p feedback_selector. The output of
  /// the PidController is clipped to be within the specified bounds. Note
  /// that using input limits along with integral gain constant may cause the
  /// integrator to windup.
  static ConnectResult ConnectControllerWithInputSaturation(
      const InputPort<T>& plant_input,
      const OutputPort<T>& plant_output,
      const MatrixX<double>& feedback_selector,
      const Eigen::VectorXd& Kp, const Eigen::VectorXd& Ki,
      const Eigen::VectorXd& Kd, const VectorX<T>& min_plant_input,
      const VectorX<T>& max_plant_input, DiagramBuilder<T>* builder);

  /// Creates a PidController with input saturation and uses @p builder to
  /// connect @p plant_input and @p plant_output from an existing plant. The
  /// plant's full state is used for feedback. The output of the PidController
  /// is clipped to be within the specified bounds. Note that using input
  /// limits along with integral gain constant may cause the integrator to
  /// windup.
  static ConnectResult ConnectControllerWithInputSaturation(
      const InputPort<T>& plant_input,
      const OutputPort<T>& plant_output,
      const Eigen::VectorXd& Kp, const Eigen::VectorXd& Ki,
      const Eigen::VectorXd& Kd, const VectorX<T>& min_plant_input,
      const VectorX<T>& max_plant_input, DiagramBuilder<T>* builder);

 private:
  // A helper function for the constructors. This is necessary to avoid seg
  // faults caused by simultaneously moving the plant and calling methods on
  // the plant when one constructor delegates to another constructor.
  void Initialize(std::unique_ptr<System<T>> plant,
                  const MatrixX<double>& feedback_selector,
                  const Eigen::VectorXd& Kp, const Eigen::VectorXd& Ki,
                  const Eigen::VectorXd& Kd);

  System<T>* plant_{nullptr};
  const int state_output_port_index_;
};

}  // namespace controllers
}  // namespace systems
}  // namespace drake
