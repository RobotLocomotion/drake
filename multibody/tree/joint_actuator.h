#pragma once

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/multibody/tree/multibody_element.h"
#include "drake/multibody/tree/multibody_forces.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"
#include "drake/multibody/tree/multibody_tree_topology.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {

// Forward declaration for JointActuator<T>.
template<typename T> class Joint;

/// The %JointActuator class is mostly a simple bookkeeping structure to
/// represent an actuator acting on a given Joint.
/// It helps to flag whether a given Joint is actuated or not so that
/// MultibodyTree clients can apply forces on actuated joints through their
/// actuators, see AddInOneForce().
///
/// @tparam_default_scalar
template <typename T>
class JointActuator final
    : public MultibodyElement<JointActuator, T, JointActuatorIndex> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(JointActuator)

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

  /// Returns the name of the actuator.
  const std::string& name() const { return name_; }

  /// Returns a reference to the joint actuated by this %JointActuator.
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
  ///   must be in the range `0 <= joint_dof < num_velocities()` or otherwise
  ///   this method will throw an exception.
  /// @param[in] joint_tau
  ///   Generalized force corresponding to the degree of freedom indicated by
  ///   `joint_dof` to be added into `forces`. Refere to the specific Joint
  ///   sub-class documentation for details on the meaning and units for this
  ///   degree of freedom.
  /// @param[out] forces
  ///   On return, this method will add force `tau` for the degree of
  ///   freedom `joint_dof` into the output `forces`. This method aborts if
  ///   `forces` is `nullptr` or if `forces` doest not have the right sizes to
  ///   accommodate a set of forces for the model to which this actuator
  ///   belongs.
  void AddInOneForce(
      const systems::Context<T>& context,
      int joint_dof,
      const T& tau,
      MultibodyForces<T>* forces) const;

  /// Gets the actuation values for `this` actuator from the actuation vector u
  /// for the entire model.
  /// @return a reference to a nv-dimensional vector, where nv is the number
  ///         of velocity variables of joint().
  const Eigen::Ref<const VectorX<T>> get_actuation_vector(
      const VectorX<T>& u) const {
    DRAKE_DEMAND(u.size() == this->get_parent_tree().num_actuated_dofs());
    return u.segment(topology_.actuator_index_start, joint().num_velocities());
  }

  /// Given the actuation values u_instance for `this` actuator, this method
  /// sets the actuation vector u for the entire MultibodyTree model
  /// to which this actuator belongs to.
  /// @param[in] u_instance
  ///   Actuation values for `this` actuator. It must be of size equal to the
  ///   number of degrees of freedom of the actuated Joint, see
  ///   Joint::num_velocities(). For units and sign conventions refer to the
  ///   specific Joint sub-class documentation.
  /// @param[out] u
  ///   The vector containing the actuation values for the entire MultibodyTree
  ///   model to which `this` actuator belongs to.
  /// @throws std::exception if
  ///   `u_instance.size() != this->joint().num_velocities()`.
  /// @throws std::exception if u is nullptr.
  /// @throws std::exception if
  ///   `u.size() != this->get_parent_tree().num_actuated_dofs()`.
  void set_actuation_vector(
      const Eigen::Ref<const VectorX<T>>& u_instance,
      EigenPtr<VectorX<T>> u) const;

  /// Returns the actuator effort limit.
  double effort_limit() const { return effort_limit_; }

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
  const T& rotor_inertia(const systems::Context<T>& context) const {
    return context.get_numeric_parameter(rotor_inertia_parameter_index_)[0];
  }

  /// Returns the associated gear ratio value for this actuator, stored in
  /// `context`.
  /// See @ref reflected_inertia.
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
  T calc_reflected_inertia(const systems::Context<T>& context) const {
    const T& rho = gear_ratio(context);
    const T& Ir = rotor_inertia(context);
    return rho * rho * Ir;
  }
  /// @} <!-- Reflected Inertia -->

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
  /// @endcond

 private:
  // Allow different specializations to access each other's private constructor
  // for scalar conversion.
  template <typename U> friend class JointActuator;

  // Private constructor used for cloning.
  JointActuator(const std::string& name, JointIndex joint_index,
                double effort_limit, double rotor_inertia, double gear_ratio)
      : name_(name),
        joint_index_(joint_index),
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
  // At MultibodyTree::Finalize() time, each actuator retrieves its topology
  // from the parent MultibodyTree.
  void DoSetTopology(const internal::MultibodyTreeTopology&) final;

  // Implementation for MultibodyElement::DoDeclareParameters().
  void DoDeclareParameters(
      internal::MultibodyTreeSystem<T>* tree_system) final {
    // Declare parent classes' parameters
    MultibodyElement<JointActuator, T, JointActuatorIndex>::DoDeclareParameters(
        tree_system);
    rotor_inertia_parameter_index_ = this->DeclareNumericParameter(
        tree_system,
        systems::BasicVector<T>(Vector1<T>(default_rotor_inertia_)));
    gear_ratio_parameter_index_ = this->DeclareNumericParameter(
        tree_system, systems::BasicVector<T>(Vector1<T>(default_gear_ratio_)));
  }

  // The actuator's unique name in the MultibodyTree model
  std::string name_;

  // The index of the joint on which this actuator acts.
  JointIndex joint_index_;

  // Actuator effort limit. It must be greater than 0.
  double effort_limit_;

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

  // The topology of this actuator. Only valid post- MultibodyTree::Finalize().
  internal::JointActuatorTopology topology_;
};

}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::JointActuator)
