#pragma once

#include <memory>
#include <stdexcept>
#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_deprecated.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/controllers/inverse_dynamics.h"
#include "drake/systems/controllers/pid_controller.h"
#include "drake/systems/controllers/state_feedback_controller_interface.h"
#include "drake/systems/framework/diagram.h"

namespace drake {
namespace systems {
namespace controllers {

// N.B. Inheritance order must remain fixed for pydrake (#9243).
/**
 * A state feedback controller that uses a PidController to generate desired
 * accelerations, which are then converted into torques using InverseDynamics.
 * More specifically, the output of this controller is:
 * `torque = inverse_dynamics(q, v, vd_d)`,
 * where `vd_d = kp(q* - q) + kd(v* - v) + ki int(q* - q) + vd*`.
 * `q` and `v` stand for the generalized position and velocity, and `vd` is
 * the generalized acceleration. `*` indicates reference values.
 *
 * @system
 * name: InverseDynamicsController
 * input_ports:
 * - estimated_state
 * - desired_state
 * - desired_acceleration*
 * output_ports:
 * - force
 * @endsystem
 *
 * This controller always has a BasicVector input port for estimated robot state
 * `(q, v)`, a BasicVector input port for reference robot state `(q*, v*)` and
 * a BasicVector output port for computed torque `torque`. A constructor flag
 * can be set to track reference acceleration `vd*` as well. When set, a
 * BasicVector input port is also declared, and it's content is used as `vd*`.
 * When unset, `vd*` is be treated as zero.
 *
 * Note that this class assumes the robot is fully actuated, its position
 * and velocity have the same dimension, and it does not have a floating base.
 * If violated, the program will abort. This controller was not designed for
 * closed-loop systems: the controller accounts for neither constraint forces
 * nor actuator forces applied at loop constraints. Use on such systems is not
 * recommended.
 *
 * @see InverseDynamics for an accounting of all forces incorporated into the
 *      inverse dynamics computation.
 *
 * @tparam_double_only
 * @ingroup control_systems
 */
template <typename T>
class InverseDynamicsController : public Diagram<T>,
                                  public StateFeedbackControllerInterface<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(InverseDynamicsController)

  /**
   * Constructs an inverse dynamics controller for the given `plant` model.
   * The %InverseDynamicsController holds an internal, non-owned reference to
   * the MultibodyPlant object so you must ensure that `plant` has a longer
   * lifetime than `this` %InverseDynamicsController.
   * @param plant The model of the plant for control.
   * @param kp Position gain.
   * @param ki Integral gain.
   * @param kd Velocity gain.
   * @param has_reference_acceleration If true, there is an extra BasicVector
   * input port for `vd*`. If false, `vd*` is treated as zero, and no extra
   * input port is declared.
   * @pre `plant` has been finalized (plant.is_finalized() returns `true`).
   * @throws std::exception if
   *  - The plant is not finalized (see MultibodyPlant::Finalize()).
   *  - The number of generalized velocities is not equal to the number of
   *    generalized positions.
   *  - The model is not fully actuated.
   *  - Vector kp, ki and kd do not all have the same size equal to the number
   *    of generalized positions.
   *
   * @pydrake_mkdoc_identifier{5args_referenced_plant}
   */
  InverseDynamicsController(
      const multibody::MultibodyPlant<T>& plant,
      const VectorX<double>& kp,
      const VectorX<double>& ki,
      const VectorX<double>& kd,
      bool has_reference_acceleration);

  /**
   * Constructs an inverse dynamics controller and takes the ownership of the
   * input `plant`.
   *
   * @pydrake_mkdoc_identifier{5args_owned_plant}
   */
  InverseDynamicsController(std::unique_ptr<multibody::MultibodyPlant<T>> plant,
                            const VectorX<double>& kp,
                            const VectorX<double>& ki,
                            const VectorX<double>& kd,
                            bool has_reference_acceleration);

  ~InverseDynamicsController() override;

  /**
   * Sets the integral part of the PidController to @p value.
   * @p value must be a column vector of the appropriate size.
   */
  void set_integral_value(Context<T>* context,
                          const Eigen::Ref<const VectorX<T>>& value) const;

  /**
   * Returns the input port for the reference acceleration.
   */
  const InputPort<T>& get_input_port_desired_acceleration() const {
    DRAKE_DEMAND(has_reference_acceleration_);
    DRAKE_DEMAND(input_port_index_desired_acceleration_ >= 0);
    return Diagram<T>::get_input_port(input_port_index_desired_acceleration_);
  }

  /**
   * Returns the input port for the estimated state.
   */
  const InputPort<T>& get_input_port_estimated_state() const final {
    return this->get_input_port(input_port_index_estimated_state_);
  }

  /**
   * Returns the input port for the desired state.
   */
  const InputPort<T>& get_input_port_desired_state() const final {
    return this->get_input_port(input_port_index_desired_state_);
  }

  /**
   * Returns the output port for computed control.
   */
  const OutputPort<T>& get_output_port_control() const final {
    return this->get_output_port(output_port_index_control_);
  }

  /**
   * Returns a constant pointer to the MultibodyPlant used for control.
   */
  const multibody::MultibodyPlant<T>* get_multibody_plant_for_control() const {
    return multibody_plant_for_control_;
  }

 private:
  void SetUp(const VectorX<double>& kp, const VectorX<double>& ki,
             const VectorX<double>& kd);

  const std::unique_ptr<multibody::MultibodyPlant<T>>
      owned_plant_for_control_{};
  const multibody::MultibodyPlant<T>* multibody_plant_for_control_{nullptr};
  PidController<T>* pid_{nullptr};
  const bool has_reference_acceleration_{false};
  InputPortIndex input_port_index_estimated_state_;
  InputPortIndex input_port_index_desired_state_;
  InputPortIndex input_port_index_desired_acceleration_;
  OutputPortIndex output_port_index_control_;
};

}  // namespace controllers
}  // namespace systems
}  // namespace drake
