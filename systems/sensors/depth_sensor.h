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
#include "drake/systems/framework/system.h"
#include "drake/systems/rendering/pose_vector.h"
#include "drake/systems/sensors/depth_sensor_output.h"
#include "drake/systems/sensors/depth_sensor_specification.h"

namespace drake {
namespace systems {
namespace sensors {

/// A simple model of an ideal depth sensor. Example real-world depth sensors
/// include Lidar, IR, sonar, etc. The depth measurements are taken at evenly
/// spaced pitch angles and yaw angles. Ray casting is used to obtain the depth
/// measurements and all pitch / yaw combinations in a single resulting depth
/// image are obtained at a single effective point in time. Thus, this sensor
/// does *not* model aliasing effects due to the time spent scanning vertically
/// and horizontally. For the ray casting, DepthSensor uses collision models of
/// RigidBodyTree.
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
/// 1. yaw   - The horizontal scan angle that rotates about this sensor's base
///            frame's +Z axis using the right-hand rule. When the pitch is
///            zero, a yaw of zero results in this sensor measuring down the
///            base frame's +X axis.
/// 2. pitch - The vertical scan angle that rotates about this sensor's base
///            frames's -Y axis using using the right-hand rule. In other
///            words, when the pitch increases from zero to PI / 2, this
///            sensor's optical frame is tilted to point upward. When the yaw
///            is zero, a pitch of zero results in this sensor measuring down
///            the base frame's +X axis.
///
/// To summarize, the location from which this sensor's rays emanate is
/// (0, 0, 0) in the base frame. When both the yaw and pitch are zero, this
/// sensor's base and optical frames are identical. The yaw causes the optical
/// frame to rotate about the base frame's +Z axis, while the pitch causes it to
/// rotate about this sensor's -Y axis.
///
/// This system has two output ports. The first contains the sensed values
/// stored in a DepthSensorOutput object. For each pitch, there is a fixed
/// number of yaw values as specified by num_yaw(). Each of these vector
/// of yaw values that share the same pitch are contiguous in the output vector.
/// In other words, here is pseudocode describing this sensor's output vector:
///
/// <pre>
///  for i in 0 to num_pitch():
///    for j in 0 to num_yaw():
///      output_vector[i * num_yaw() + j] ==
///          [depth value when yaw   = min_yaw()   + j * yaw_increment() and
///                            pitch = min_pitch() + i * pitch_increment()]
/// </pre>
///
/// If nothing is detected between min_range and max_range, an invalid value of
/// DepthSensor::kTooFar is provided. Is something is detected but the
/// distance is less than  the sensor's minimum sensing range, a value of
/// DepthSensor::kTooClose is provided.
///
/// Sensing errors can be expressed using a distance value of
/// DepthSensorOutput::GetErrorDistance(), and detected using
/// DepthSensorOutput::IsError() or DepthSensorOutput::IsValid(). It is not
/// expected to occur in this system's output because it models an ideal sensor
/// in which sensing errors do not occur. Sensing errors are expected to be used
/// by non-ideal depth sensors.
///
/// The second output port contains a PoseVector, which is `X_WS`, i.e., the
/// transform from this sensor's frame to the world frame. It is useful for
/// converting this sensor's point cloud into the world frame.
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
  /// @param[in] specification The specifications of this sensor.
  ///
  DepthSensor(const std::string& name, const RigidBodyTree<double>& tree,
              const RigidBodyFrame<double>& frame,
              const DepthSensorSpecification& specification);

  /// Returns the RigidBodyTree that this sensor is sensing.
  const RigidBodyTree<double>& get_tree() const { return tree_; }

  /// Returns this sensor's frame, which specifies its location and orientation
  /// in the RigidBodyTree.
  const RigidBodyFrame<double>& get_frame() const { return frame_; }

  /// Returns this sensor's specification.
  const DepthSensorSpecification& get_specification() { return specification_; }

  /// Returns the number of pixel rows in the resulting depth sensor output.
  /// This is equal to parameter DepthSensorSpecification::num_pitch_values
  /// that's passed into the constructor.
  int get_num_pitch() const { return specification_.num_pitch_values(); }

  /// Returns the number of pixel columns in the resulting depth sensor output.
  /// This is equal to parameter DepthSensorSpecification::num_yaw_values
  /// that's passed into the constructor.
  int get_num_yaw() const { return specification_.num_yaw_values(); }

  /// Returns the size of the system's output, which equals the product of
  /// num_pitch() and num_yaw().
  int get_num_depth_readings() const {
    return specification_.num_depth_readings();
  }

  /// Returns the input port containing the generalized state of the
  /// RigidBodyTree.
  const InputPort<double>& get_rigid_body_tree_state_input_port()
      const;

  /// Returns the state output port, which contains the sensor's
  /// sensed values.
  const OutputPort<double>& get_sensor_state_output_port() const;

  /// Returns the `X_WS` output port, which contains the
  /// transform from this sensor's frame to the world frame.
  const OutputPort<double>& get_pose_output_port() const;

  friend std::ostream& operator<<(std::ostream& out,
                                  const DepthSensor& depth_sensor);

 private:
  // These are calculators for the depth data and sensor pose outputs.
  void CalcDepthOutput(const Context<double>& context,
                       DepthSensorOutput<double>* data_output) const;

  void CalcPoseOutput(const Context<double>& context,
                      rendering::PoseVector<double>* pose_output) const;

  // Allocator for our CacheEntry. Returns a KinematicsCache suitable for
  // the RigidBodyTree we're using.
  KinematicsCache<double> AllocateKinematicsCache() const;

  // Calculator for the CacheEntry. Updates the KinematicsCache to reflect
  // the current q's at the input port.
  void CalcKinematics(const Context<double>& context,
                      KinematicsCache<double>*) const;

  // The depth sensor will cast a ray with its start point at (0,0,0) in the
  // sensor's base frame (as defined by get_frame()). Its end, also in the
  // sensor's base frame, is computed by this method and stored in
  // raycast_endpoints_ at the time of construction. raycast_endpoints_ is only
  // computed once at the time of construction since the end points are constant
  // in the sensor's base frame.
  void PrecomputeRaycastEndpoints();

  // Applies the min / max range of the sensor. Any measurement that is less
  // than the minimum or greater than the maximum is set to an invalid value.
  // This is so users of this sensor can distinguish between an object at the
  // maximum sensing distance and not detecting any object within the sensing
  // range.
  void ApplyLimits(VectorX<double>* dists) const;

  const std::string name_;
  const RigidBodyTree<double>& tree_;
  const RigidBodyFrame<double> frame_;
  const DepthSensorSpecification specification_;
  int input_port_index_{};
  int depth_output_port_index_{};
  int pose_output_port_index_{};
  const CacheEntry* kinematics_cache_entry_{};

  // A cache of where a depth measurement ray endpoint would be if the maximum
  // range were achieved. This is cached to avoid repeated allocation and
  // computation.
  Eigen::Matrix3Xd raycast_endpoints_;
};

}  // namespace sensors
}  // namespace systems
}  // namespace drake
