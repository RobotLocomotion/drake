#pragma once

#include <memory>
#include <string>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/rigid_body_frame.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/output_port_value.h"
#include "drake/systems/framework/system.h"
#include "drake/systems/framework/system_port_descriptor.h"
#include "drake/systems/sensors/depth_sensor_specification.h"

namespace drake {
namespace systems {
namespace sensors {

/// A simple model of an ideal depth sensor. Example real-world depth sensors
/// include Lidar, IR, sonar, etc. The depth measurements are taken at evenly
/// spaced pixel rows (pitch angles) and columns (yaw angles). Ray casting is
/// used to obtain the depth measurements and all pitch / yaw combinations in a
/// single resulting depth image are obtained at a single effective point in
/// time. Thus, this sensor does *not* model aliasing effects due to the time
/// spent scanning vertically and horizontally.
///
/// There are two frames associated with this sensor: its base frame and its
/// optical frame. This sensor's specification and configuration are defined in
/// terms of this sensor's base frame. The base frame's origin defines the point
/// from which this sensor's ray casts originate. Its +X, +Y, and +Z axes are
/// pointing towards this sensor's "forward," "left", and "up" directions,
/// respectively.
///
/// This sensor's optical frame's origin is the same as the base frame's origin.
/// Its +X axis defines where a particular ray cast is pointing. There are two
/// angles associated with a depth measurement, as described below. Both angles
/// are defined in terms of the sensor's base frame. Together they define a
/// transform between the base frame and optical frame.
///
///   1. yaw   - The horizontal scan angle that rotates about this sensor's base
///              frame's +Z axis using the right-hand rule. When the pitch is
///              zero, a yaw of zero results in this sensor measuring down the
///              base frame's +X axis.
///   2. pitch - The vertical scan angle that rotates about this sensor's base
///              frames's -Y axis using using the right-hand rule. In other
///              words, when the pitch increases from zero to PI / 2, this
///              sensor's optical frame is tilted to point upward. When the yaw
///              is zero, a pitch of zero results in this sensor measuring down
///              the base frame's +X axis.
///
/// To summarize, the location from which this sensor's rays emanate is
/// (0, 0, 0) in the base frame. When both the yaw and pitch are zero, this
/// sensor's base and optical frames are identical. The yaw causes the optical
/// frame to rotate about the base frame's +Z axis, while the pitch causes it to
/// rotate about this sensor's -Y axis.
///
/// This system has one output port containing the sensed values. It is a
/// vector representation of a depth image. For each pitch, there is a fixed
/// number of yaw values as specified as num_pixel_cols(). Each of these vector
/// of yaw values that share the same pitch are contiguous in the output vector.
/// In other words, here is some pseudocode describing this sensor's output
/// vector:
///
/// <pre>
///  for i in 0 to num_pixel_rows():
///    for j in 0 to num_pixel_cols():
///      output_vector[i * num_pixel_cols() + j] ==
///          [depth value when yaw   = min_yaw()   + j * yaw_increment() and
///                            pitch = min_pitch() + i * pitch_increment()]
/// </pre>
///
/// If nothing is detected in between min_range and max_range, an invalid value
/// of DepthSensor::kTooFar is provided. Is something is detected but the
/// distance is less than  the sensor's minimum sensing range, a value of
/// DepthSensor::kTooClose is provided.
///
/// DepthSensor::kError is defined for use when a sensing error occurs. It is
/// not expected to occur in this system's output because it models an ideal
/// sensor in which sensing errors do not occur. It is expected to be used by
/// non-ideal depth sensors.
///
/// @ingroup sensor_systems
///
/// @see DepthSensorOutput
/// @see DepthSensorSpecification
///
class DepthSensor : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DepthSensor)

  /// A %DepthSensor constructor.
  ///
  /// @param[in] name The name of the depth sensor. This can be any value, but
  /// should typically be unique among all sensors attached to a particular
  /// model instance.
  ///
  /// @param[in] tree The RigidBodyTree containing the geometric configuration
  /// of the world. This parameter is aliased by a class member variable. Thus,
  /// its lifespan must exceed that of this class' instance.
  ///
  /// @param[in] frame The frame to which this depth sensor is attached.
  ///
  /// @param[in] specification The specifications of this sensor. An alias of
  /// this specification is stored as a class member variable, meaning its
  /// lifetime must exceed the lifetime of the %DepthSensor object created by
  /// this constructor.
  ///
  DepthSensor(const std::string& name, const RigidBodyTree<double>& tree,
              const RigidBodyFrame<double>& frame,
              const DepthSensorSpecification& specification);

  /// Returns the name of this sensor. The name can be any user-specified value.
  const std::string& get_name() const { return name_; }

  /// Returns the RigidBodyTree that this sensor is sensing.
  const RigidBodyTree<double>& get_tree() const { return tree_; }

  /// Returns this sensor's frame, which specifies its location and orientation
  /// in the RigidBodyTree.
  const RigidBodyFrame<double>& get_frame() const { return frame_; }

  /// Returns this sensor's specification.
  const DepthSensorSpecification& get_specification() { return specification_; }

  /// Returns the number of pixel rows in the resulting depth sensor output.
  /// This is equal to parameter `num_pitch_values` that's passed into the
  /// constructor.
  int get_num_pixel_rows() const { return specification_.num_pitch_values(); }

  /// Returns the number of pixel columns in the resulting depth sensor output.
  /// This is equal to parameter `num_yaw_values` that's passed into the
  /// constructor.
  int get_num_pixel_cols() const { return specification_.num_yaw_values(); }

  /// Returns the size of the system's output, which equals the product of
  /// num_pixel_rows() and num_pixel_cols().
  int get_num_depth_readings() const {
    return specification_.num_depth_readings();
  }

  /// Returns a descriptor of the input port containing the generalized state of
  /// the RigidBodyTree.
  const InputPortDescriptor<double>& get_rigid_body_tree_state_input_port()
      const;

  /// Returns a descriptor of the state output port, which contains the sensor's
  /// sensed values.
  const OutputPortDescriptor<double>& get_sensor_state_output_port() const;

  friend std::ostream& operator<<(std::ostream& out,
                                  const DepthSensor& depth_sensor);

 protected:
  /// Outputs the depth information.
  void DoCalcOutput(const systems::Context<double>& context,
                    systems::SystemOutput<double>* output) const override;

 private:
  // The depth sensor will cast a ray with its start point at (0,0,0) in the
  // sensor's base frame (as defined by get_frame()). Its end, also in the
  // sensor's base frame, is computed by this method and stored in
  // raycast_endpoints_ at the time of construction. raycast_endpoints_ is only
  // computed once at the time of construction since the end points are constant
  // in the sensor's base frame.
  void PrecomputeRaycastEndpoints();

  const std::string name_;
  const RigidBodyTree<double>& tree_;
  const RigidBodyFrame<double> frame_;
  const DepthSensorSpecification& specification_;
  int input_port_index_{};
  int output_port_index_{};

  // A cache of where a depth measurement ray endpoint would be if the maximum
  // range were achieved. This is cached to avoid repeated allocation.
  Eigen::Matrix3Xd raycast_endpoints_;
};

}  // namespace sensors
}  // namespace systems
}  // namespace drake
