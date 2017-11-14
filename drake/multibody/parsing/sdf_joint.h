#pragma once

#include <limits>
#include <map>
#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace parsing {

/// A representation of a `<joint>` element in an SDF file.
/// For details on the specification of joints, including conventions and
/// default values, please refer to the documentation for the
/// <a href="http://sdformat.org/spec?ver=1.6&elem=joint">
/// &lt;joint&gt; element</a>.
class SDFJoint {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SDFJoint);

  /// Creates a new joint object specification with the given `joint_name`.
  /// Per SDF specification, `joint_name` must be unique within the scope of the
  /// joint's model. Uniqueness is **not** enforced by %SDFJoint.
  ///
  /// @param[in] joint_name
  ///   The name of this joint as specified in the
  ///   <a href="http://sdformat.org/spec?ver=1.6&elem=joint#joint_name">
  ///   &lt;name&gt; element</a> documentation.
  /// @param[in] parent_link_name
  ///   The name of the parent link as defined in the
  ///   <a href="http://sdformat.org/spec?ver=1.6&elem=joint#joint_parent">
  ///   &lt;parent&gt; element</a> documentation.
  /// @param[in] child_link_name
  ///   The name of the parent link as defined in the
  ///   <a href="http://sdformat.org/spec?ver=1.6&elem=joint#joint_child">
  ///   &lt;child&gt; element</a> documentation.
  /// @param[in] joint_type
  ///   The type of the joint. E.g: revolute, prismatic. See documentation on
  ///   the <a href="http://sdformat.org/spec?ver=1.6&elem=joint#joint_type">
  ///   &lt;type&gt; element</a> for details.
  SDFJoint(
      const std::string& joint_name,
      const std::string& parent_link_name,
      const std::string& child_link_name,
      const std::string& joint_type)
      : name_(joint_name),
        parent_link_name_(parent_link_name), child_link_name_(child_link_name),
        joint_type_(joint_type) {}

  /// Returns the name of `this` joint.
  const std::string& name() const { return name_; }

  /// Returns the name of `this` joint's parent link.
  const std::string& parent_link() const { return parent_link_name_; }

  /// Returns the name of `this` joint's child link.
  const std::string& child_link() const { return child_link_name_; }

  /// Returns the name of the type of `this` joint.
  const std::string& joint_type() const { return joint_type_; }

  /// Returns `true` if `this` joint type has an "axis" property, as it is the
  /// case for revolute and prismatic joint types.
  bool JointHasAxis() const {
    return joint_type_ == "revolute" || joint_type_ == "prismatic";
  }

  /// Returns the axis of this joint expressed in the joint frame.
  /// For details on how this frame is defined refer to the documenatation for
  /// the <a href="http://sdformat.org/spec?ver=1.6&elem=joint#joint_axis">
  /// &lt;axis&gt; element</a>.
  /// @warning This method aborts for joint types that do not have an axis
  /// property.
  const Vector3<double>& get_axis() const {
    DRAKE_DEMAND(JointHasAxis());
    return axis_;
  }

  /// Sets the axis for this joint, expressed in the joint frame.
  /// @warning This method aborts for joint types that do not have an axis
  /// property.
  /// @sa get_axis()
  void set_axis(const Vector3<double>& axis) {
    DRAKE_DEMAND(JointHasAxis());
    axis_ = axis;
  }

 private:
  // Name of the root frame of this cache.
  std::string name_;
  std::string parent_link_name_;
  std::string child_link_name_;
  std::string joint_type_;
  Vector3<double> axis_{
      Vector3<double>::Constant(std::numeric_limits<double>::quiet_NaN())};
};

}  // namespace parsing
}  // namespace multibody
}  // namespace drake
