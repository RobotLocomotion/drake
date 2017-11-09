#pragma once

#include <limits>
#include <map>
#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/parsing/sdf/framed_isometry3.h"

namespace drake {
namespace multibody {
namespace multibody_tree {
namespace parsing {

/// A representation of a `<joint>` entry in an SDF file.
class SDFJoint {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SDFJoint);
  /// Creates a new joint object specification with the given `joint_name`.
  /// Per SDF specification, `joint_name` must be unique within the scope of the
  /// joint's model.
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
}  // namespace multibody_tree
}  // namespace multibody
}  // namespace drake
