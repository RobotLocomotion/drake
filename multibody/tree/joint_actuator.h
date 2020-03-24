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
                double effort_limit)
      : name_(name), joint_index_(joint_index), effort_limit_(effort_limit) {}

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

  // The actuator's unique name in the MultibodyTree model
  std::string name_;

  // The index of the joint on which this actuator acts.
  JointIndex joint_index_;

  // Actuator effort limit. It must be greater than 0.
  double effort_limit_;

  // The topology of this actuator. Only valid post- MultibodyTree::Finalize().
  internal::JointActuatorTopology topology_;
};

}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::JointActuator)
