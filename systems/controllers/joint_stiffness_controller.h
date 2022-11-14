#pragma once

#include <memory>
#include <stdexcept>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {
namespace controllers {

// TODO(russt): Consider adding a feed-forward torque, τ_ff.  It's not a
// priority since it's trivial to add it to the output of this system with an
// Adder block.
/**
 * Implements a joint-space stiffness controller of the form
 * <pre>
 *   τ_control = −τ_g(q) − τ_app + kp⊙(q_d − q) + kd⊙(v_d − v)
 * </pre>
 * where `Kp` and `Kd` are the joint stiffness and damping coefficients,
 * respectively, `τ_g(q)` is the vector of generalized forces due to gravity,
 * and `τ_app` contains applied forces from force elements added to the
 * multibody model (this can include damping, springs, etc. See
 * MultibodyPlant::CalcForceElementsContribution()).  `q_d` and `v_d` are the
 * desired (setpoint) values for the multibody positions and velocities,
 * respectively. `kd` and `kp` are taken as vectors, and ⊙ represents
 * elementwise multiplication.
 *
 * The goal of this controller is to produce a closed-loop dynamics that
 * resembles a spring-damper dynamics at the joints around the setpoint:
 * <pre>
 *   M(q)v̇ + C(q,v)v + kp⊙(q - q_d) + kd⊙(v - v_d) = τ_ext,
 * </pre>
 * where `M(q)v̇ + C(q,v)v` are the original multibody mass and Coriolis terms,
 * and `τ_ext` are any external generalized forces that can arise, e.g. from
 * contact forces.
 *
 * The controller currently requires that plant.num_positions() ==
 * plant.num_velocities() == plant.num_actuated_dofs().
 *
 * @system
 * name: JointStiffnessController
 * input_ports:
 * - estimated_state
 * - desired_state
 * output_ports:
 * - generalized_force
 * @endsystem
 *
 * Note that the joint impedance control as implemented on Kuka's iiwa and
 * Franka' Panda is best modeled as a stiffness controller (unless one were to
 * model the actual elastic joints and rotor-inertia shaping). See
 * https://manipulation.csail.mit.edu/force.html for more details.
 *
 * @tparam_default_scalar
 * @ingroup control_systems
 */
template <typename T>
class JointStiffnessController final : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(JointStiffnessController)

  /**
   * Constructs the JointStiffnessController system.
   *
   * @param plant Reference to the multibody plant model. The life span of @p
   * plant must be at least as long as that of this instance.
   * @pre The plant must be finalized (i.e., plant.is_finalized() must return
   * `true`).
   * @pre plant.num_positions() == plant.num_velocities() ==
   * plant.num_actuated_dofs() == kp.size() == kd.size()
   */
  JointStiffnessController(const multibody::MultibodyPlant<T>& plant,
                           const Eigen::Ref<const Eigen::VectorXd>& kp,
                           const Eigen::Ref<const Eigen::VectorXd>& kd);

  /**
   * Constructs the JointStiffnessController system and takes the ownership of
   * the input `plant`.
   * @pre The plant must be finalized (i.e., plant.is_finalized() must return
   * `true`).
   * @pre plant.num_positions() == plant.num_velocities() ==
   * plant.num_actuated_dofs() == kp.size() == kd.size()
   *
   * @exclude_from_pydrake_mkdoc{This overload is not bound.}
   */
  explicit JointStiffnessController(
      std::unique_ptr<multibody::MultibodyPlant<T>> plant,
      const Eigen::Ref<const Eigen::VectorXd>& kp,
      const Eigen::Ref<const Eigen::VectorXd>& kd);

  // Scalar-converting copy constructor.  See @ref system_scalar_conversion.
  template <typename U>
  explicit JointStiffnessController(const JointStiffnessController<U>& other);

  ~JointStiffnessController() override;

  // TODO(russt): Add support for safety limits. The iiwa driver will fault if
  // the desired state and the estimated state are too far apart. We should
  // support that (optional) feature here -- it's not very iiwa-specific.

  /**
   * Returns the input port for the estimated state.
   */
  const InputPort<T>& get_input_port_estimated_state() const {
    return this->get_input_port(input_port_index_estimated_state_);
  }

  /**
   * Returns the input port for the desired state.
   */
  const InputPort<T>& get_input_port_desired_state() const {
    return this->get_input_port(input_port_index_desired_state_);
  }

  /**
   * Returns the output port for the generalized forces implementing the
   * control. */
  const OutputPort<T>& get_output_port_generalized_force() const {
    return this->get_output_port(output_port_index_force_);
  }

  /**
   * Returns a constant pointer to the MultibodyPlant used for control.
   */
  const multibody::MultibodyPlant<T>& get_multibody_plant() const {
    return *plant_;
  }

 private:
  // Other constructors delegate to this private constructor.
  JointStiffnessController(
      std::unique_ptr<multibody::MultibodyPlant<T>> owned_plant,
      const multibody::MultibodyPlant<T>* plant,
      const Eigen::Ref<const Eigen::VectorXd>& kp,
      const Eigen::Ref<const Eigen::VectorXd>& kd);

  template <typename> friend class JointStiffnessController;

  // This is the calculator method for the output port.
  void CalcOutputForce(const Context<T>& context,
                       BasicVector<T>* force) const;

  // Methods for updating cache entries.
  void SetMultibodyContext(const Context<T>&, Context<T>*) const;
  void CalcMultibodyForces(const Context<T>&,
                           multibody::MultibodyForces<T>*) const;

  const std::unique_ptr<multibody::MultibodyPlant<T>> owned_plant_{};
  const multibody::MultibodyPlant<T>* const plant_;

  int input_port_index_estimated_state_{0};
  int input_port_index_desired_state_{0};
  int output_port_index_force_{0};

  Eigen::VectorXd kp_, kd_;

  drake::systems::CacheIndex applied_forces_cache_index_;
  drake::systems::CacheIndex plant_context_cache_index_;
};

#ifdef DRAKE_DOXYGEN_CXX
/** Drake does not yet offer a joint impedance controller, which would use
 * feedback to shape the stiffness, damping, *and inertia* of the closed-loop
 * system.  We do however offer JointStiffnessController which uses feedback to
 * shape the closed-loop stiffness and damping.  Most modern control stacks,
 * such as the JointImpedanceControl mode on Kuka's iiwa and Franka's Panda, are
 * actually better modeled as stiffness control (unless one is also modeling the
 * details of the elastic joints and rotor-interia shaping).
 *
 * See https://manipulation.csail.mit.edu/force.html for more details.
 * @see JointStiffnessController.
 *
 * @ingroup control_systems
 */
class JointImpedanceController {};
#endif

}  // namespace controllers
}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::controllers::JointStiffnessController)
