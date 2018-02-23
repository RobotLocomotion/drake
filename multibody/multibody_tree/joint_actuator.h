#pragma once

#include <memory>
#include <string>
#include <vector>

#include "drake/common/autodiff.h"
#include "drake/common/drake_copyable.h"
#include "drake/multibody/multibody_tree/multibody_tree_element.h"
#include "drake/multibody/multibody_tree/multibody_tree_indexes.h"
#include "drake/multibody/multibody_tree/multibody_tree_topology.h"

namespace drake {
namespace multibody {

// Forward declaration for JointActuator<T>.
template<typename T> class Joint;

/// The %JointActuator class is mostly a simple bookkeeping structure to
/// represent an actuator acting on a given Joint. When added to a MultibodyTree
/// model, a %JointActuator gets assigned a JointActuatorIndex, which can be
/// retrieved with JointActuator::get_index(). The %JointActuator class
/// allows mapping this JointActuatorIndex to the Joint on which it actuates,
/// which can be retrieved with JointActuator::joint().
///
/// @tparam T The scalar type. Must be a valid Eigen scalar.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - AutoDiffXd
/// - symbolic::Expression
///
/// They are already available to link against in the containing library.
/// No other values for T are currently supported.
template <typename T>
class JointActuator final
    : public MultibodyTreeElement<JointActuator<T>, JointActuatorIndex> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(JointActuator)

  /// Creates an actuator for `joint` with the given `name`.
  /// The name must be unique withing the given MultibodyTree model. This is
  /// guaranteed by MultibodyTree::AddJointActuator().
  JointActuator(const std::string& name, const Joint<T>& joint);

  /// Returns the name of the actuator.
  const std::string& get_name() const { return name_; }

  /// Returns the joint actuated by this %JointActuator.
  const Joint<T>& joint() const;

  /// @cond
  // For internal use only.
  // NVI to DoCloneToScalar() templated on the scalar type of the new clone to
  // be created. This method is intended to be called by
  // MultibodyTree::CloneToScalar().
  template <typename ToScalar>
  std::unique_ptr<JointActuator<ToScalar>> CloneToScalar(
  const MultibodyTree<ToScalar>& cloned_tree) const {
    return DoCloneToScalar(cloned_tree);
  }
  /// @endcond

 private:
  // Allow different specializations to access each other's private constructor
  // for scalar conversion.
  template <typename U> friend class JointActuator;

  // Private constructor used for cloning.
  JointActuator(const std::string& name, JointIndex joint_index)
      : name_(name), joint_index_(joint_index) {}

  // Helper to clone an actuator (templated on T) to an actuator templated on
  // `double`.
  std::unique_ptr<JointActuator<double>> DoCloneToScalar(
      const MultibodyTree<double>& tree_clone) const;

  // Helper to clone an actuator (templated on T) to an actuator templated on
  // `AutoDiffXd`.
  std::unique_ptr<JointActuator<AutoDiffXd>> DoCloneToScalar(
      const MultibodyTree<AutoDiffXd>& tree_clone) const;

  // Implementation for MultibodyTreeElement::DoSetTopology().
  // At MultibodyTree::Finalize() time, each actuator retrieves its topology
  // from the parent MultibodyTree.
  void DoSetTopology(const MultibodyTreeTopology&) final {
    // So far this initial implementation has no topology counterpart.
  }

  // The actuator's unique name in the MultibodyTree model
  std::string name_;

  // The index of the joint on which this actuator acts.
  JointIndex joint_index_;
};

}  // namespace multibody
}  // namespace drake
