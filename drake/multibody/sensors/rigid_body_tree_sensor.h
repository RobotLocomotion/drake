#pragma once

#include <memory>
#include <string>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/multibody/rigid_body_frame.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace multibody {

/// The parent class of all sensors that sense the RigidBodyTree.
///
/// @ingroup sensor_systems
///
class RigidBodyTreeSensor : public systems::LeafSystem<double> {
 public:
  /// A constructor that initializes member variable values.
  ///
  /// @param[in] tree The RigidBodyTree containing the geometric configuration
  /// of the world. This parameter is aliased by a class member variable. Thus,
  /// its lifespan must exceed that of this class' instance.
  ///
  /// @param[in] sensor_name The name of the depth sensor. This can be any
  /// value, but should typically be unique among all sensors attached to a
  /// particular model instance.
  ///
  /// @param[in] frame The frame to which this depth sensor is attached.
  ///
  RigidBodyTreeSensor(const RigidBodyTree<double>& tree,
                      const std::string& sensor_name,
                      const RigidBodyFrame<double>& frame);

  // Non-copyable.

  /// @name Deleted Copy/Move Operations
  /// IdealDepthSensor is neither copyable nor moveable.
  ///@{
  explicit RigidBodyTreeSensor(const RigidBodyTreeSensor&) = delete;
  RigidBodyTreeSensor& operator=(const RigidBodyTreeSensor&) = delete;
  ///@}

  /// Returns the name of this sensor. The name can be any user-specified value.
  const std::string& get_name() const;

  /// Returns the RigidBodyTree that this sensor is sensing.
  const RigidBodyTree<double>& get_tree() const;

  /// Returns this sensor's frame, which specifies its location and orientation
  /// in the RigidBodyTree.
  const RigidBodyFrame<double>& get_frame() const;

 private:
  const RigidBodyTree<double>& tree_;
  const std::string sensor_name_;
  const RigidBodyFrame<double> frame_;
};

}  // namespace multibody
}  // namespace drake
