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
/// For details on the specification of links, including conventions and default
/// values, please refer to the documentation for the
/// <a href="http://sdformat.org/spec?ver=1.6&elem=joint">
/// &lt;joint&gt; element</a>.
class SDFJoint {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SDFJoint);

  /// Creates a new joint object specification with the given `joint_name`.
  /// Per SDF specification, `joint_name` must be unique within the scope of the
  /// joint's model.
  /// @param[in] parent_link_name
  ///   The name of the parent link as defined in the
  ///   <a href="http://sdformat.org/spec?ver=1.6&elem=joint#joint_parent">
  ///   &lt;parent&gt; element</a> documentation.
  /// @param[in] child_link_name
  ///   The name of the parent link as defined in the
  ///   <a href="http://sdformat.org/spec?ver=1.6&elem=joint#joint_child">
  ///   &lt;child&gt; element</a> documentation.
  SDFJoint(
      const std::string& joint_name,
      const std::string& parent_link_name,
      const std::string& child_link_name,
      const std::string& joint_type) :
      name_(joint_name),
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

  /// Returns the axis of this joint expressed in the joint frame.
  const Vector3<double>& get_axis() const {
    DRAKE_DEMAND(JointHasAxis());
    return axis_;
  }

  void set_axis(const Vector3<double>& axis) {
    DRAKE_DEMAND(JointHasAxis());
    axis_ = axis;
  }

 private:
  bool JointHasAxis() const {
    return joint_type_ == "revolute" || joint_type_ == "prismatic";
  }

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
