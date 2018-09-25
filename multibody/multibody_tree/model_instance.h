#pragma once

#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/joint_actuator.h"
#include "drake/multibody/multibody_tree/mobilizer.h"
#include "drake/multibody/multibody_tree/multibody_tree_element.h"
#include "drake/multibody/multibody_tree/multibody_tree_indexes.h"

namespace drake {
namespace multibody {

/// @file
/// @anchor model_instance
/// Model instance information for multibody trees.
///
/// A MultiBodyTree is composed of a number of MultibodyTreeElement items.  In
/// more complex trees, these items will be loaded from multiple models
/// (e.g. robot arm, attached gripper, free bodies being manipulated).  Each
/// model may have a different controller or observer, so the ability to view
/// the state or command actuation for each model independently can be useful.
/// For this reason, elements are associated with a ModelInstanceIndex to
/// determine which model an element belongs to.  In most cases, the allocation
/// of model instances and assignment of elements to the appropriate model
/// instance will be handled by the parser loading the model.
///
/// There are two special ModelElementIndex values.  The world body is always
/// ModelInstanceIndex 0, and ModelInstanceIndex 1 is reserved for all elements
/// with no explicit model instance.  This is generally only relevant for
/// elements created programmatically (and for which a model instance is not
/// explicitly specified), as model parsers should handle creating model
/// elements as needed.
///
/// For different types of MultibodyTreeElement, the model instance is
/// sometimes specified explicitly, and sometimes inferred when the
/// element is created.  The current convention is:
///
/// * Body: Specified at creation
/// * BodyNode: Same as the associated Body
/// * Frame: Same as the associated Body
/// * Joint: Same as the child frame
/// * JointActuator: Same as the associated Joint
/// * Mobilizer:
///   * Same as the joint (if created from a joint)
///   * Same as the body (for free floating bodies)
///   * When creating mobilizers for other reasons (e.g. for unit tests) the
///     author should take care to choose a reasonable model instance.
/// * ForceElement: Depends on the type of element
///   * UniformGravityFieldElement: uses the world model instance

// Note:
//   static global variables are strongly discouraged by the C++ style guide:
// https://google.github.io/styleguide/cppguide.html#Static_and_Global_Variables
// For this reason, we create and return an instance of ModelInstanceIndex
// instead of using a static variable.

/// Returns the model instance containing the *world* body.  For
/// every MultibodyTree the **world** body _always_ has this unique
/// model instance and it is always zero (as described in #3088).
inline ModelInstanceIndex world_model_instance() {
  return ModelInstanceIndex(0);
}

/// Returns the model instance which contains all tree elements with
/// no explicit model instance specified.
inline ModelInstanceIndex default_model_instance() {
  return ModelInstanceIndex(1);
}

namespace internal {

/// @tparam T The scalar type. Must be a valid Eigen scalar.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - AutoDiffXd
/// - symbolic::Expression
template <typename T>
class ModelInstance :
      public MultibodyTreeElement<ModelInstance<T>, ModelInstanceIndex> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ModelInstance)

  explicit ModelInstance(ModelInstanceIndex index)
      : MultibodyTreeElement<ModelInstance<T>, ModelInstanceIndex>(index) {}

  int num_positions() const { return num_positions_; }
  int num_velocities() const { return num_velocities_; }
  int num_states() const { return num_positions_ + num_velocities_; }
  int num_actuated_dofs() const { return num_actuated_dofs_; }

  void add_mobilizer(const Mobilizer<T>* mobilizer) {
    num_positions_ += mobilizer->num_positions();
    num_velocities_ += mobilizer->num_velocities();
    mobilizers_.push_back(mobilizer);
  }

  void add_joint_actuator(const JointActuator<T>* joint_actuator) {
    num_actuated_dofs_ += joint_actuator->joint().num_velocities();
    joint_actuators_.push_back(joint_actuator);
  }

  /// Given the actuation values @p u_instance for all actuators in `this` model
  /// instance, this method sets the portion of the actuation vector @p u (which
  /// is the actuation vector for the entire MultibodyTee) corresponding to the
  /// actuators for this model instance.
  /// @param[in] u_instance Actuation values for the actuators. It must be of
  ///   size equal to the number of degrees of freedom of all of the actuated
  ///   joints in this model instance.
  /// @param[out] u
  ///   The vector containing the actuation values for the entire MultibodyTree
  ///   model to which `this` actuator belongs to. It must be of size equal to
  ///   the number of degrees of freedom of all of the actuated joints in the
  ///   entire MultibodyTree model.
  /// @throws std::logic_error if `u_instance` is not of size equal to the
  ///         number of degrees of freedom of all of the actuated joints in this
  ///         model or `u` is not of size equal to the number of degrees of
  ///         freedom of all of the actuated joints in the entire MultibodyTree
  ///         model to which `this` actuator belongs to.
  void set_actuation_vector(
      const Eigen::Ref<const VectorX<T>>& u_instance,
      EigenPtr<VectorX<T>> u) const;

  /// Returns a const Eigen expression of the vector of generalized positions
  /// for `this` model instance from a vector `q_array` of generalized
  /// positions for the entire MultibodyTree model.
  /// @throws std::logic_error if `q_array` is not of size
  ///         MultibodyTree::num_positions().
  VectorX<T> get_positions_from_array(
      const Eigen::Ref<const VectorX<T>>& q_array) const;

  /// Sets the vector of generalized positions for `this` model instance in
  /// the relevant locations of an array that corresponds to the positions for
  /// the entire MultibodyTree model. Elements of the array that do not
  /// correspond to `this` are not altered. Note that calling
  /// `set_positions_in_array(model_q, q_array)` causes
  /// `get_positions_from_array(q_array)` to yield `model_q`.
  /// @throws std::logic_error if `q_array` is not of size
  ///         MultibodyTree::num_positions() or `model_q` is not of size
  ///         num_positions().
  void set_positions_in_array(
      const Eigen::Ref<const VectorX<T>>& model_q,
      EigenPtr<VectorX<T>> q_array) const;

  /// Returns a const Eigen expression of the vector of generalized velocities
  /// for `this` mobilizer from a vector `v_array` of generalized velocities for
  /// the entire MultibodyTree model.
  /// @throws std::logic_error if `v_array` is not of size
  ///         MultibodyTree::num_velocities().
  VectorX<T> get_velocities_from_array(
      const Eigen::Ref<const VectorX<T>>& v_array) const;

  /// Sets the vector of generalized velocities for `this` model instance in
  /// the relevant locations of an array corresponding to the velocities for the
  /// entire MultibodyTree model. Elements of the array that do not
  /// correspond to `this` are not altered. Note that calling
  /// `set_velocities_in_array(model_v, v_array)` causes
  /// `get_velocities_from_array(v_array)` to yield `model_v`.
  /// @throws std::logic_error if `v_array` is not of size
  ///         MultibodyTree::num_velocities() or `model_v` is not of size
  ///         num_velocities().
  void set_velocities_in_array(
      const Eigen::Ref<const VectorX<T>>& model_v,
      EigenPtr<VectorX<T>> v_array) const;

 private:
  void DoSetTopology(const MultibodyTreeTopology&) final {}

  int num_positions_{0};
  int num_velocities_{0};
  int num_actuated_dofs_{0};

  std::vector<const Mobilizer<T>*> mobilizers_;
  std::vector<const JointActuator<T>*> joint_actuators_;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
