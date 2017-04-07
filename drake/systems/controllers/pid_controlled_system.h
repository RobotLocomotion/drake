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
#include "drake/systems/primitives/demultiplexer.h"
#include "drake/systems/primitives/gain.h"
#include "drake/systems/primitives/matrix_gain.h"

namespace drake {
namespace systems {

/// A system that encapsulates a PidController and a controlled System (a.k.a
/// the "plant").
///
/// The passed in plant must meet the following properties:
///
/// * Input port zero must be all of the control inputs (size N). When the plant
///   is a dynamics model, this is typically the generalized effort (e.g., force
///   or torque) command.
///
/// * Output port zero must be of size N * 2, where the first N elements are the
///   position states of the plant, and the second N elements are the velocity
///   states of the plant.
///
/// The resulting PidControlledSystem has two input ports and one output port
/// with the following properties:
///
/// * Input port zero is the feed forward control input to the plant.
///
/// * Input port one is the desired position and velocity state of the plant.
///
/// * The output port is the current state of the plant (it is the direct
///   pass-through of the plant's output port).
///
/// Some of the constructors include a parameter called `feedback_selector`.
/// This is a MatrixGain that is used to determine which elements in the plant's
/// output port zero are used as part of the feedback signal to the PID
/// controller. Let `|u|` equal the size of the plant's input port zero, and
/// `|y|` equal the size of plant's output port zero. The gain matrix in
/// parameter `feedback_selector` must have dimensions of `(|y|, 2 * |u|)`. This
/// matrix typically contains at most one `1` in each row, and zeros everywhere
/// else. A `1` at location `[m, n]` indicates that element `m` of the plant's
/// output port zero is fed back as element `n` of the feedback signal to the
/// PID controller.
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

  /// A constructor where the gains are scalar values and all of the plant's
  /// output port zero is part of the feedback signal.
  ///
  /// @param[in] plant The system to be controlled. This must not be `nullptr`.
  /// @param[in] Kp the proportional constant.
  /// @param[in] Ki the integral constant.
  /// @param[in] Kd the derivative constant.
  PidControlledSystem(std::unique_ptr<System<T>> plant, double Kp, double Ki,
                      double Kd);

  /// A constructor where the gains are vector values and all of the plant's
  /// output port zero is part of the feedback signal. The length of the gain
  /// vectors must equal the size of the plant's output port zero, which is
  /// double the size of the plant's input port zero.
  ///
  /// @param[in] plant The system to be controlled. This must not be `nullptr`.
  ///
  /// @param[in] Kp the proportional vector constant.
  ///
  /// @param[in] Ki the integral vector constant.
  ///
  /// @param[in] Kd the derivative vector constant.
  PidControlledSystem(std::unique_ptr<System<T>> plant,
                      const Eigen::VectorXd& Kp, const Eigen::VectorXd& Ki,
                      const Eigen::VectorXd& Kd);

  /// A constructor where the gains are scalar values and some of the plant's
  /// output is part of the feedback signal as specified by
  /// @p feedback_selector.
  ///
  /// @param[in] plant The system to be controlled. This must not be `nullptr`.
  /// @param[in] feedback_selector The system that selects which part of the
  /// plant's output port zero is fed back to the PID controller. For semantic
  /// details of this parameter, see this class's description. This parameter
  /// can be `nullptr`, in which case the entire output port zero is fed back.
  /// @param[in] Kp the proportional constant.
  /// @param[in] Ki the integral constant.
  /// @param[in] Kd the derivative constant.
  PidControlledSystem(std::unique_ptr<System<T>> plant,
                      std::unique_ptr<MatrixGain<T>> feedback_selector,
                      double Kp, double Ki, double Kd);

  /// A constructor where the gains are vector values and some of the plant's
  /// output is part of the feedback signal as specified by
  /// @p feedback_selector. The length of the gain vectors must equal the size
  /// of the feedback selector's output port zero, which is also equal to double
  /// the size of the plant's input port zero.
  ///
  /// @param[in] plant The system to be controlled. This must not be `nullptr`.
  ///
  /// @param[in] feedback_selector The system that selects which part of the
  /// plant's output port zero is fed back to the PID controller. For semantic
  /// details and required dimensions of this parameter, see this class's
  /// description. This parameter can be `nullptr`, in which case the plant's
  /// entire output port zero is fed back to the PID controller.
  ///
  /// @param[in] Kp the proportional vector constant.
  ///
  /// @param[in] Ki the integral vector constant.
  ///
  /// @param[in] Kd the derivative vector constant.
  PidControlledSystem(std::unique_ptr<System<T>> plant,
                      std::unique_ptr<MatrixGain<T>> feedback_selector,
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
  /// @p plant_output from an existing plant, adding additional systems
  /// (adders, multiplexers, gains, etc.) as needed.
  static ConnectResult ConnectController(
      const InputPortDescriptor<T>& plant_input,
      const OutputPort<T>& plant_output,
      std::unique_ptr<MatrixGain<T>> feedback_selector,
      const Eigen::VectorXd& Kp, const Eigen::VectorXd& Ki,
      const Eigen::VectorXd& Kd, DiagramBuilder<T>* builder);

  /// Creates a PidController with input saturation and uses @p builder to
  /// connect @p plant_input and @p plant_output from an existing plant, adding
  /// additional systems (adders, multiplexers, gains, saturation etc.) as
  /// needed. Note that using input limits along with integral gain constant
  /// may result in integral windup effects.
  static ConnectResult ConnectControllerWithInputSaturation(
      const InputPortDescriptor<T>& plant_input,
      const OutputPort<T>& plant_output,
      std::unique_ptr<MatrixGain<T>> feedback_selector,
      const Eigen::VectorXd& Kp, const Eigen::VectorXd& Ki,
      const Eigen::VectorXd& Kd, const VectorX<T>& min_plant_input,
      const VectorX<T>& max_plant_input, DiagramBuilder<T>* builder);

 private:
  // A helper function for the constructors. This is necessary to avoid seg
  // faults caused by simultaneously moving the plant and calling methods on
  // the plant when one constructor delegates to another constructor.
  void Initialize(std::unique_ptr<System<T>> plant,
                  std::unique_ptr<MatrixGain<T>> feedback_selector,
                  const Eigen::VectorXd& Kp, const Eigen::VectorXd& Ki,
                  const Eigen::VectorXd& Kd);

  System<T>* plant_{nullptr};
};

}  // namespace systems
}  // namespace drake
