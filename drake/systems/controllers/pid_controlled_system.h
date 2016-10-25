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
  /// @param[in] plant The system to be controlled.
  /// @param[in] Kp the proportional constant.
  /// @param[in] Ki the integral constant.
  /// @param[in] Kd the derivative constant.
  PidControlledSystem(std::unique_ptr<System<T>> plant,
                      const T& Kp, const T& Ki, const T& Kd);
  ~PidControlledSystem() override;

  System<T>* plant() { return plant_; }

  /// Sets @p context to a default state in which the positions and
  /// velocities are all zero.  The integral of the controller is also
  /// set to zero.
  void SetDefaultState(Context<T>* context) const;

 private:
  System<T>* plant_;
  PidController<T>* controller_;

  // Takes as input the plant's error state vector and outputs separate position
  // and velocity state error vectors. These outputs are then inputted into the
  // PID controller.
  Demultiplexer<T>* error_demux_;

  // Inverts the PID controller's output command. The output of this system is
  // inputted into plant_input_.
  Gain<T>* controller_inverter_;

  // Inverts the PidControlledSystem input containing the desired state of the
  // plant. The output is inputted into state_minus_target_.
  Gain<T>* error_inverter_;

  // Subtracts the desired state of the plant from the current state of the
  // plant. The resulting vector is inputted into error_demux_.
  Adder<T>* state_minus_target_;

  // Adds the PID controller's output command with the provided feed forward
  // command. The resulting command is then inputted into plant_.
  Adder<T>* plant_input_;
};

}  // namespace systems
}  // namespace drake
