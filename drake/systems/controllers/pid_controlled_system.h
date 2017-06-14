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

/// A system that encapsulates a PidController and a controlled System (a.k.a
/// the "plant").
///
/// The passed in plant must meet the following properties:
///
/// * Input port zero must be all of the control inputs (size U). When the plant
///   is a dynamics model, this is typically the generalized effort (e.g., force
///   or torque) command.
///
/// * Output port zero must be of size 2 * Q, where the first Q elements are the
///   position states of the plant, and the second Q elements are the velocity
///   states of the plant. Q >= U.
///
/// The resulting PidControlledSystem has two input ports and one output port
/// with the following properties:
///
/// * Input port zero is the feed forward control (size U), which will be added
///   onto the output of the PID controller. The sum is sent to the plant's
///   input.
///
/// * Input port one is the desired *controlled* states (2 * U) of the plant,
///   where the first half are the *controlled* positions, and the second half
///   are the *controlled* velocities.
///
/// * The output port is the current state of the plant (it is the direct
///   pass-through of the plant's output port).
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
/// @tparam T The vector element type, which must be a valid Eigen scalar.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - AutoDiffXd
///
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
  PidControlledSystem(std::unique_ptr<System<T>> plant, double Kp, double Ki,
                      double Kd);

  /// @p plant full state is used for feedback control, and the vectorized gains
  /// are specified by @p Kp, @p Kd and @p Ki.
  ///
  /// @param[in] plant The system to be controlled. This must not be `nullptr`.
  /// @param[in] Kp the proportional vector constant.
  /// @param[in] Ki the integral vector constant.
  /// @param[in] Kd the derivative vector constant.
  PidControlledSystem(std::unique_ptr<System<T>> plant,
                      const Eigen::VectorXd& Kp, const Eigen::VectorXd& Ki,
                      const Eigen::VectorXd& Kd);

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
  PidControlledSystem(std::unique_ptr<System<T>> plant,
                      const MatrixX<double>& feedback_selector,
                      double Kp, double Ki, double Kd);

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
  PidControlledSystem(std::unique_ptr<System<T>> plant,
                      const MatrixX<double>& feedback_selector,
                      const Eigen::VectorXd& Kp, const Eigen::VectorXd& Ki,
                      const Eigen::VectorXd& Kd);

  ~PidControlledSystem() override;

  System<T>* plant() { return plant_; }

  /// @return the input port for the feed forward control input.
  const InputPortDescriptor<T>& get_control_input_port() const {
    return this->get_input_port(0);
  }

  /// @return the input port for the desired position/velocity state.
  const InputPortDescriptor<T>& get_state_input_port() const {
    return this->get_input_port(1);
  }

  /// The return type of ConnectController.
  struct ConnectResult {
    /// The feed forward control input.
    const InputPortDescriptor<T>& control_input_port;
    /// The feedback state input.
    const InputPortDescriptor<T>& state_input_port;
  };

  /// Creates a PidController and uses @p builder to connect @p plant_input and
  /// @p plant_output from an existing plant. The controlled states are selected
  /// by @p feedback_selector.
  static ConnectResult ConnectController(
      const InputPortDescriptor<T>& plant_input,
      const OutputPort<T>& plant_output,
      const MatrixX<double>& feedback_selector,
      const Eigen::VectorXd& Kp, const Eigen::VectorXd& Ki,
      const Eigen::VectorXd& Kd, DiagramBuilder<T>* builder);

  /// Creates a PidController and uses @p builder to connect @p plant_input and
  /// @p plant_output from an existing plant. The plant's full state is used for
  /// feedback.
  static ConnectResult ConnectController(
      const InputPortDescriptor<T>& plant_input,
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
      const InputPortDescriptor<T>& plant_input,
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
      const InputPortDescriptor<T>& plant_input,
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
};

}  // namespace systems
}  // namespace drake
