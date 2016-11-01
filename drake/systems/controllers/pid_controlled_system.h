#pragma once

#include <memory>

#include "drake/common/drake_export.h"
#include "drake/systems/controllers/pid_controller.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/system.h"
#include "drake/systems/framework/primitives/adder.h"
#include "drake/systems/framework/primitives/constant_vector_source.h"
#include "drake/systems/framework/primitives/demultiplexer.h"
#include "drake/systems/framework/primitives/gain.h"
#include "drake/systems/framework/primitives/matrix_gain.h"

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
class DRAKE_EXPORT PidControlledSystem : public Diagram<T> {
 public:
  /// A constructor where the gains are scalar values and all of the plant's
  /// output port zero is part of the feedback signal.
  ///
  /// @param[in] plant The system to be controlled. This must not be `nullptr`.
  /// @param[in] Kp the proportional constant.
  /// @param[in] Ki the integral constant.
  /// @param[in] Kd the derivative constant.
  PidControlledSystem(std::unique_ptr<System<T>> plant,
                      const T& Kp, const T& Ki, const T& Kd);

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
                      const VectorX<T>& Kp, const VectorX<T>& Ki,
                      const VectorX<T>& Kd);

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
                      const T& Kp, const T& Ki, const T& Kd);

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
                      const VectorX<T>& Kp, const VectorX<T>& Ki,
                      const VectorX<T>& Kd);

  ~PidControlledSystem() override;

  System<T>* plant() { return plant_; }

  /// Sets @p context to a default state in which the positions and
  /// velocities are all zero.  The integral of the controller is also
  /// set to zero.
  void SetDefaultState(Context<T>* context) const;

 private:
  // A helper function for the constructors. This is necessary to avoid seg
  // faults caused by simultaneously moving the plant and calling methods on the
  // plant when one constructor delegates to another constructor.
  void Initialize(
    std::unique_ptr<System<T>> plant,
    std::unique_ptr<MatrixGain<T>> feedback_selector,
    const VectorX<T>& Kp, const VectorX<T>& Ki, const VectorX<T>& Kd);

  System<T>* plant_{nullptr};
  PidController<T>* controller_{nullptr};
  MatrixGain<T>* feedback_selector_{nullptr};

  // Takes as input the plant's error state vector and outputs separate position
  // and velocity state error vectors. These outputs are then inputted into the
  // PID controller.
  Demultiplexer<T>* error_demux_{nullptr};

  // Inverts the PID controller's output command. The output of this system is
  // inputted into plant_input_.
  Gain<T>* controller_inverter_{nullptr};

  // Inverts the PidControlledSystem input containing the desired state of the
  // plant. The output is inputted into state_minus_target_.
  Gain<T>* error_inverter_{nullptr};

  // Subtracts the desired state of the plant from the current state of the
  // plant. The resulting vector is inputted into error_demux_.
  Adder<T>* state_minus_target_{nullptr};

  // Adds the PID controller's output command with the provided feed forward
  // command. The resulting command is then inputted into plant_.
  Adder<T>* plant_input_{nullptr};
};

}  // namespace systems
}  // namespace drake
