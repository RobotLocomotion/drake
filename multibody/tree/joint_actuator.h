#pragma once

#include <limits>
#include <memory>
#include <optional>
#include <string>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/multibody/tree/multibody_element.h"
#include "drake/multibody/tree/multibody_forces.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {

// Forward declaration for JointActuator<T>.
template <typename T>
class Joint;

/// PD controller gains. This enables the modeling of a simple low level PD
/// controllers, see JointActuator::set_controller_gains().
struct PdControllerGains {
  // Proportional gain of the controller.
  double p{0.0};
  // Derivative gain of the controller.
  double d{0.0};
};

/// The %JointActuator class is mostly a simple bookkeeping structure to
/// represent an actuator acting on a given Joint.
/// It helps to flag whether a given Joint is actuated or not so that
/// MultibodyTree clients can apply forces on actuated joints through their
/// actuators, see AddInOneForce().
///
/// @tparam_default_scalar
template <typename T>
class JointActuator final : public MultibodyElement<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(JointActuator);

  /// Creates an actuator for `joint` with the given `name`.
  /// The name must be unique within the given multibody model. This is
  /// enforced by MultibodyPlant::AddJointActuator().
  /// @param[in] name
  ///   A string with a name identifying `this` actuator.
  /// @param[in] joint
  ///   The `joint` that the created actuator will act on.
  /// @param[in] effort_limit
  ///   The maximum effort for the actuator. It must be strictly positive,
  ///   otherwise an std::exception is thrown. If +∞, the actuator has no limit,
  ///   which is the default. The effort limit has physical units in accordance
  ///   to the joint type it actuates. For instance, it will have units of
  ///   N⋅m (torque) for revolute joints while it will have units of N (force)
  ///   for prismatic joints.
  JointActuator(const std::string& name, const Joint<T>& joint,
                double effort_limit = std::numeric_limits<double>::infinity());

  ~JointActuator() final;

  /// Returns this element's unique index.
  JointActuatorIndex index() const {
    return this->template index_impl<JointActuatorIndex>();
  }

  /// Returns the name of the actuator.
  const std::string& name() const { return name_; }

  /// Returns a reference to the joint actuated by this %JointActuator.
  /// @throws std::exception if this element is not associated with a
  ///   MultibodyPlant.
  const Joint<T>& joint() const;

  /// Adds into `forces` a force along one of the degrees of freedom of the
  /// Joint actuated by `this` actuator.
  /// The meaning for this degree of freedom, sign conventions and even its
  /// dimensional units depend on the specific joint sub-class being actuated.
  /// For a RevoluteJoint for instance, `joint_dof` can only be 0 since revolute
  /// joints's motion subspace only has one degree of freedom, while the units
  /// of `tau` are those of torque (N⋅m in the MKS system of units). For
  /// multi-dof joints please refer to the documentation provided by specific
  /// joint sub-classes regarding the meaning of `joint_dof`.
  ///
  /// @param[in] context
  ///   The context storing the state and parameters for the model to which
  ///   `this` joint belongs.
  /// @param[in] joint_dof
  ///   Index specifying one of the degrees of freedom for this joint. The index
  ///   must be in the range `0 <= joint_dof < num_inputs()` or otherwise
  ///   this method will throw an exception.
  /// @param[in] tau
  ///   Generalized force corresponding to the degree of freedom indicated by
  ///   `joint_dof` to be added into `forces`. Refer to the specific Joint
  ///   sub-class documentation for details on the meaning and units for this
  ///   degree of freedom.
  /// @param[out] forces
  ///   On return, this method will add force `tau` for the degree of
  ///   freedom `joint_dof` into the output `forces`. This method aborts if
  ///   `forces` is `nullptr` or if `forces` doest not have the right sizes to
  ///   accommodate a set of forces for the model to which this actuator
  ///   belongs.
  void AddInOneForce(const systems::Context<T>& context, int joint_dof,
                     const T& tau, MultibodyForces<T>* forces) const;

  /// Gets the actuation values for `this` actuator from the actuation vector u
  /// for the entire plant model.
  /// @return a reference to a nv-dimensional vector, where nv is the number
  ///   of velocity variables of joint().
  /// @throws std::exception if this element is not associated with a
  ///   MultibodyPlant.
  const Eigen::Ref<const VectorX<T>> get_actuation_vector(
      const VectorX<T>& u) const {
    DRAKE_THROW_UNLESS(this->has_parent_tree());
    DRAKE_DEMAND(u.size() == this->get_parent_tree().num_actuated_dofs());
    return u.segment(actuator_dof_start_, joint().num_velocities());
  }

  /// Given the actuation values `u_actuator` for `this` actuator, updates the
  /// actuation vector `u` for the entire multibody model to which this actuator
  /// belongs to.
  /// @param[in] u_actuator
  ///   Actuation values for `this` actuator. It must be of size equal to
  ///   num_inputs(). For units and sign conventions refer to the specific Joint
  ///   sub-class documentation.
  /// @param[in,out] u
  ///   Actuation values for the entire plant model to which `this` actuator
  ///   belongs to. The actuation value in `u` for `this` actuator must be found
  ///   at offset input_start(). Only values corresponding to this actuator are
  ///   changed.
  /// @throws std::exception if
  ///   `u_actuator.size() != this->num_inputs()`.
  /// @throws std::exception if u is nullptr.
  /// @throws std::exception if this element is not associated with a
  ///   MultibodyPlant.
  /// @throws std::exception if
  ///   `u.size() != this->GetParentPlant().num_actuated_dofs()`.
  void set_actuation_vector(const Eigen::Ref<const VectorX<T>>& u_actuator,
                            EigenPtr<VectorX<T>> u) const;

  /// Returns the index to the first element for this joint actuator / within
  /// the vector of actuation inputs for the full multibody / system.
  /// Returns -1 if this %JointActuator hasn't been added to a MultibodyPlant.
  int input_start() const;

  /// Returns the number of inputs associated with this actuator. This is
  /// always the number of degrees of freedom of the actuated joint.
  int num_inputs() const;

  // TODO(russt): This should be a vector (not a double), and we should have
  //  lower and upper limits (not require them to be symmetric).

  /// Returns the actuator effort limit.
  double effort_limit() const { return effort_limit_; }

  /// Sets the actuator effort limit. (To clear the limit, set it to +∞.)
  void set_effort_limit(double effort_limit);

  // Don't let clang-format wrap long @image lines.
  // clang-format off
  /// @anchor reflected_inertia
  /// @name                 Reflected Inertia
  ///
  /// The %JointActuator class offers the ability to model the effects of
  /// reflected inertia for revolute and prismatic joints via an approximation
  /// commonly used in robotics (see [Featherstone, 2008]).
  /// Reflected inertia is an approximate method for accounting for the
  /// mass/inertia contributions to kinetic energy and equations of motion for
  /// effects of a spinning motor's rotor (and any additional shafts/gears)
  /// inside a gearmotor. This "quick and dirty" approximation does not depend
  /// on the internals of the gearbox (e.g., whether there is a single stage or
  /// multistage gear, whether there are additional shafts/gears, or the
  /// relative position or orientation of the outboard shaft with respect to the
  /// motor rotor's shaft. Note: Reflected inertia **does not** account for
  /// mass/inertia of the body attached to the gearmotor's output shaft (labeled
  /// S in the figure below). The value of reflected inertia is a function of
  /// the gear ratio and the rotor inertia of the modeled motor. %JointActuator
  /// exposes these two values as parameters (see SetRotorInertia() and
  /// SetGearRatio()). Reflected inertia has units of kg for prismatic joints
  /// and units of kg⋅m² for revolute joints. A zero value indicates reflected
  /// inertia is not modeled. This value is used as part of an approximation of
  /// a rotor's inertial effects in a geared motor. It should be noted that the
  /// approximation is reasonable for high gear ratios but less so for small
  /// gear ratios (see [Featherstone, 2008], Chapter 9.6 on gears).
  ///
  /// @image html drake/multibody/tree/images/GearBoxSchematic.png width=50%
  /// <!-- NOLINTNEXTLINE(whitespace/line_length) -->
  /// @image html drake/multibody/tree/images/GearedMotorAPiqselsComCC0.jpg width=50%
  ///
  /// #### Actuated revolute joints
  /// For an actuator driving a revolute joint, the reflected inertia can be
  /// estimated from the rotational inertia Iᵣ of the actuator's rotor about
  /// its axis of rotation and from the dimensionless gear ratio ρ.
  /// To define gear ratio ρ, consider a gear-motor combination that has a
  /// stator/rigid case A, motor rotor R, and output shaft B. Rotor R spins
  /// relative to stator A with angular speed wR and shaft B spins relative to
  /// stator A with angular speed wB. The gear ratio is defined ρ ≝ wR / wB.
  /// Typically, ρ >> 1. For this gear-motor, reflected inertia Iᵣᵢ = ρ² ⋅ Iᵣ.
  ///
  /// #### Actuated prismatic joints
  /// To define the gear ratio ρ for prismatic joints, consider a gear-motor
  /// combination that has a stator/rigid case A, motor rotor R, and translating
  /// output shaft B. Rotor R spins relative to stator A with angular speed wR
  /// and shaft B translates relative to stator A with translational speed vB.
  /// The gear ratio is defined ρ ≝ wR / vB (units of 1/m). Typically, ρ >> 1.
  /// For the gear-motor here, reflected inertia Iᵣᵢ = ρ² ⋅ Iᵣ (units of kg).
  ///@{
  // clang-format on

  /// Gets the default value for this actuator's rotor inertia.
  /// See @ref reflected_inertia.
  double default_rotor_inertia() const { return default_rotor_inertia_; }

  /// Gets the default value for this actuator's gear ratio.
  /// See @ref reflected_inertia.
  double default_gear_ratio() const { return default_gear_ratio_; }

  /// Sets the default value for this actuator's rotor inertia.
  /// See @ref reflected_inertia.
  void set_default_rotor_inertia(double rotor_inertia) {
    default_rotor_inertia_ = rotor_inertia;
  }

  /// Sets the default value for this actuator's gear ratio.
  /// See @ref reflected_inertia.
  void set_default_gear_ratio(double gear_ratio) {
    default_gear_ratio_ = gear_ratio;
  }

  /// Returns the default value for this actuator's reflected inertia.
  /// It is calculated as ρ²⋅Iᵣ, where ρ is the default gear ratio and Iᵣ is the
  /// default rotor inertia for this actuator.
  /// See @ref reflected_inertia.
  double default_reflected_inertia() const {
    return default_gear_ratio_ * default_gear_ratio_ * default_rotor_inertia_;
  }

  /// Returns the associated rotor inertia value for this actuator, stored in
  /// `context`.
  /// See @ref reflected_inertia.
  /// Note that this ONLY depends on the Parameters in the context; it does
  /// not depend on time, input, state, etc.
  const T& rotor_inertia(const systems::Context<T>& context) const {
    return context.get_numeric_parameter(rotor_inertia_parameter_index_)[0];
  }

  /// Returns the associated gear ratio value for this actuator, stored in
  /// `context`.
  /// See @ref reflected_inertia.
  /// Note that this ONLY depends on the Parameters in the context; it does
  /// not depend on time, input, state, etc.
  const T& gear_ratio(const systems::Context<T>& context) const {
    return context.get_numeric_parameter(gear_ratio_parameter_index_)[0];
  }

  /// Sets the associated rotor inertia value for this actuator in `context`.
  /// See @ref reflected_inertia.
  void SetRotorInertia(systems::Context<T>* context,
                       const T& rotor_inertia) const {
    context->get_mutable_numeric_parameter(rotor_inertia_parameter_index_)[0] =
        rotor_inertia;
  }

  /// Sets the associated gear ratio value for this actuator in `context`.
  /// See @ref reflected_inertia.
  void SetGearRatio(systems::Context<T>* context, const T& gear_ratio) const {
    context->get_mutable_numeric_parameter(gear_ratio_parameter_index_)[0] =
        gear_ratio;
  }

  /// Calculates the reflected inertia value for this actuator in `context`.
  /// See @ref reflected_inertia.
  /// Note that this ONLY depends on the Parameters in the context; it does
  /// not depend on time, input, state, etc.
  T calc_reflected_inertia(const systems::Context<T>& context) const {
    const T& rho = gear_ratio(context);
    const T& Ir = rotor_inertia(context);
    return rho * rho * Ir;
  }
  /// @} <!-- Reflected Inertia -->

  /// @anchor pd_controlled_joint_actuator
  /// @name                 PD Controlled Actuators
  ///
  /// Refer to @ref mbp_actuation "Actuation" for further details on the
  /// modeling of PD controlled actuators.
  ///@{

  // TODO(amcastro-tri): Place gains in the context as parameters.

  /// Set controller gains for this joint actuator.
  /// This enables the modeling of a simple PD controller of the form:
  ///   ũ = -Kp⋅(q − qd) - Kd⋅(v − vd) + u_ff
  ///   u = max(−e, min(e, ũ))
  /// where qd and vd are the desired configuration and velocity for joint(), Kp
  /// and Kd are the proportional and derivative gains specified in `gains`,
  /// u_ff is the feedforward actuation and `e` corresponds to effort_limit().
  ///
  /// The gains must be finite and non-negative. Setting both gains to zero
  /// will remove the controller (has_controller() will return false).
  ///
  /// For simulation, feedforward actuation can be provided through
  /// MultibodyPlant::get_actuation_input_port(). Desired configuration and
  /// velocity are specified through
  /// MultibodyPlant::get_desired_state_input_port().
  ///
  /// PD control is currently only supported for a discrete time
  /// plant. Attempting to use non-zero gains on a continuous time plant will
  /// result in an exception.
  /// See @ref pd_controllers_and_ports for further details.
  void set_controller_gains(PdControllerGains gains);

  /// Returns `true` if any non-zero controller gains have been specified with
  /// a call to set_controller_gains().
  ///
  /// @note A controller for a given model instance can be _disarmed_ if the
  /// desired state input port for its model instance is not connected. When a
  /// PD controller is disarmed, it has no effect on the MultibodyPlant's
  /// dynamics, as if there was no PD controller (still, this method returns
  /// `true` whenever non-zero gains were set with set_controller_gains().)
  /// See @ref pd_controllers_and_ports for further details.
  bool has_controller() const { return pd_controller_gains_.has_value(); }

  /// Returns a reference to the controller gains for this actuator.
  /// @pre has_controller() is `true`.
  const PdControllerGains& get_controller_gains() const {
    DRAKE_DEMAND(has_controller());
    return *pd_controller_gains_;
  }
  /// @} <!-- PD Controlled Actuators -->

  /// @cond
  // For internal use only.
  // NVI to DoCloneToScalar() templated on the scalar type of the new clone to
  // be created. This method is intended to be called by
  // MultibodyTree::CloneToScalar().
  template <typename ToScalar>
  std::unique_ptr<JointActuator<ToScalar>> CloneToScalar(
      const internal::MultibodyTree<ToScalar>& cloned_tree) const {
    return DoCloneToScalar(cloned_tree);
  }

  void set_actuator_dof_start(int actuator_dof_start) {
    DRAKE_DEMAND(actuator_dof_start >= 0);
    actuator_dof_start_ = actuator_dof_start;
  }
  /// @endcond

 private:
  // Allow different specializations to access each other's private constructor
  // for scalar conversion.
  template <typename U>
  friend class JointActuator;

  // Private constructor used for cloning.
  JointActuator(const std::string& name, JointIndex joint_index,
                int actuator_dof_start, double effort_limit,
                double rotor_inertia, double gear_ratio)
      : name_(name),
        joint_index_(joint_index),
        actuator_dof_start_(actuator_dof_start),
        effort_limit_(effort_limit),
        default_rotor_inertia_(rotor_inertia),
        default_gear_ratio_(gear_ratio) {}

  // Helper to clone an actuator (templated on T) to an actuator templated on
  // `double`.
  std::unique_ptr<JointActuator<double>> DoCloneToScalar(
      const internal::MultibodyTree<double>& tree_clone) const;

  // Helper to clone an actuator (templated on T) to an actuator templated on
  // `AutoDiffXd`.
  std::unique_ptr<JointActuator<AutoDiffXd>> DoCloneToScalar(
      const internal::MultibodyTree<AutoDiffXd>& tree_clone) const;

  std::unique_ptr<JointActuator<symbolic::Expression>> DoCloneToScalar(
      const internal::MultibodyTree<symbolic::Expression>& tree_clone) const;

  // Implementation for MultibodyElement::DoSetTopology().
  // This is called at the end of Finalize(). We just need to record
  // that this actuator has been finalized because there are some
  // checks we can't do until then.
  void DoSetTopology() final { is_finalized_ = true; }

  // Implementation for MultibodyElement::DoDeclareParameters().
  void DoDeclareParameters(
      internal::MultibodyTreeSystem<T>* tree_system) final {
    // Sets model values to dummy values to indicate that the model values are
    // not used. This class stores the the default values of the parameters.
    rotor_inertia_parameter_index_ =
        this->DeclareNumericParameter(tree_system, systems::BasicVector<T>(1));
    gear_ratio_parameter_index_ =
        this->DeclareNumericParameter(tree_system, systems::BasicVector<T>(1));
  }

  // Implementation for MultibodyElement::DoSetDefaultParameters().
  void DoSetDefaultParameters(systems::Parameters<T>* parameters) const final {
    // Set the default rotor inertia.
    systems::BasicVector<T>& rotor_inertia_parameter =
        parameters->get_mutable_numeric_parameter(
            rotor_inertia_parameter_index_);
    rotor_inertia_parameter.set_value(Vector1<T>(default_rotor_inertia_));
    // Set the default gear ratio.
    systems::BasicVector<T>& gear_ratio_parameter =
        parameters->get_mutable_numeric_parameter(gear_ratio_parameter_index_);
    gear_ratio_parameter.set_value(Vector1<T>(default_gear_ratio_));
  }

  // The actuator's unique name in the MultibodyTree model
  std::string name_;

  // The index of the joint on which this actuator acts.
  JointIndex joint_index_;

  // This is the offset into the vector of actuated dofs at which this
  // actuator's dofs start. The number of controlled dofs starting here is
  // joint().num_velocities().
  int actuator_dof_start_{-1};

  // Actuator effort limit. It must be strictly greater than 0.
  double effort_limit_{};

  // The rotor inertia for this actuator. A zero value indicates that motor
  // inertial effects are not modeled for `this` actuator.
  double default_rotor_inertia_{0.0};

  // The gear ratio between this actuator's rotor and output shaft. The default
  // value is set to 1.0 to allow convenient sysId by just varying the single
  // rotor inertia parameter.
  double default_gear_ratio_{1.0};

  // System parameter index for `this` actuator's rotor inertia stored in a
  // context.
  systems::NumericParameterIndex rotor_inertia_parameter_index_;

  // System parameter index for `this` actuator's gear ratio stored in a
  // context.
  systems::NumericParameterIndex gear_ratio_parameter_index_;

  std::optional<PdControllerGains> pd_controller_gains_{std::nullopt};

  bool is_finalized_{false};
};

}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::JointActuator);
