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

/// A system which encapsulates a PidController and a controlled
/// System.
///
/// The passed in system must meet the following properties:
///
/// * The first input port must be all of the control inputs to the
/// * system (size N).
///
/// * The first output port must be of size N * 2, where the first N
/// * elements are the positions of the elements of the controlled
/// * system, and the second N elements are the velocities.
///
/// The resulting PidControlledSystem has two input ports and one
/// output port with the following properties:
///
/// * The first input port is the feed forward control input to the
/// * system.
///
/// * The second input port is the desired state of the system.
///
/// * The output port is the current state of the system that is being
/// * controlled (direct passthrough of the controlled system's output
/// * port).
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
  /// @param[in] system The system to be controlled.
  /// @param[in] Kp the proportional constant.
  /// @param[in] Ki the integral constant.
  /// @param[in] Kd the derivative constant.
  PidControlledSystem(std::unique_ptr<System<T>> system,
                      const T& Kp, const T& Ki, const T& Kd);
  ~PidControlledSystem() override;

  System<T>* system() { return system_; }

  /// Sets @p context to a default state in which the positions and
  /// velocities are all zero.  The integral of the controller is also
  /// set to zero.
  void SetDefaultState(Context<T>* context) const;

 private:
  System<T>* system_;
  PidController<T>* controller_;
  Demultiplexer<T>* error_demux_;
  Gain<T>* controller_inverter_;
  Gain<T>* error_inverter_;
  Adder<T>* state_minus_target_;
  Adder<T>* system_input_;
};

}  // namespace systems
}  // namespace drake
