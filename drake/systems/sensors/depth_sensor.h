#pragma once

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/multibody/rigid_body_frame.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/system.h"
#include "drake/systems/framework/system_output.h"
#include "drake/systems/framework/system_port_descriptor.h"
#include "drake/systems/sensors/depth_sensor_specification.h"

namespace drake {
namespace systems {
namespace sensors {

/// A simple model of an ideal depth sensor. Example real-world depth sensors
/// include Lidar, IR, sonar, etc. The depth measurements are taken at evenly
/// spaced pixel rows and columns. Simple ray casting is used to obtain the
/// depth measurements and all measurements of a single depth scan are obtained
/// at a single effective point in time. Thus, this sensor does *not* model
/// aliasing effects due to the time spent scanning vertically and horizontally.
///
/// The sensor is located at a single (x, y, z) point from which the
/// above-mentioned rays emanate. There are two angles associated with a depth
/// measurement:
///
///   1. theta - The horizontal scan angle that rotates about the +Z axis using
///              the right-hand rule. An angle of zero is the +X axis.
///   2. phi   - The vertical scan angle that rotates about the -Y axis using
///              using the right-hand rule. An angle of zero is the +X axis.
///
/// The sensor's frame is defined as follows: The location from which the rays
/// emanate is (0, 0, 0). When both theta and phi are zero, the sensor is
/// pointing straight down its frame's +X axis. In this configuration, the
/// sensor's frame's +Y axis points to the left and the +Z axis points up
/// relative to the sensor, per right-hand rule.
///
/// This system has one output port containing the sensed values. It is a
/// vector representation of the scan. For each value of phi, there are theta
/// values. Each of these groups of same-phi theta values are stored
/// contiguously in the output vector. In other words:
///
///  for (int i = 0; i < num_pixel_rows; ++i) {
///    for (int j = 0; j <= num_pixel_cols; ++j) {
///      output_vector[i * num_pixel_cols + j] ==
///          [depth value at theta = min_theta() + j * theta_increment() and
///           phi = min_phi() + i * phi_increment()]
///    }
///  }
///
/// If nothing is detected in between min_range and max_range, an invalid value
/// of DepthSensor::kInvalidDepthMeasurement is provided.
///
/// @ingroup sensor_systems
///
/// @see DepthSensorOutput.
/// @see DepthSensorSpecification.
///
class DepthSensor : public systems::LeafSystem<double> {
 public:
  /// The depth value when an error occurs in obtaining the measurement.
  static constexpr double kError{std::numeric_limits<double>::quiet_NaN()};

  /// The depth value when the max sensing range is exceeded.
  static constexpr double kTooFar{std::numeric_limits<double>::infinity()};

  /// The depth value when the min sensing range is violated.
  static constexpr double kTooClose{-std::numeric_limits<double>::infinity()};

  /// A constructor that initializes a DepthSensor using a separate input
  /// parameter for each dimension within the sensor specification.
  ///
  /// @param[in] name The name of the depth sensor. This can be any
  /// value, but should typically be unique among all sensors attached to a
  /// particular model instance.
  ///
  /// @param[in] tree The RigidBodyTree containing the geometric configuration
  /// of the world. This parameter is aliased by a class member variable. Thus,
  /// its lifespan must exceed that of this class' instance.
  ///
  /// @param[in] frame The frame to which this depth sensor is attached. A copy
  /// of this frame is stored as a class member variable.
  ///
  /// @param[in] min_theta The minimum horizontal (in @p frame) scan angle.
  ///
  /// @param[in] max_theta The maximum horizontal (in @p frame) scan angle.
  ///
  /// @param[in] min_phi The minimum vertical (in @p frame) scan angle.
  ///
  /// @param[in] max_phi The maximum vertical (in @p frame) scan angle.
  ///
  /// @param[in] num_theta_values The number of theta values at which depth
  /// measurements are taken. These measurements are evenly spread between
  /// @p min_theta and @p max_theta. This value must be greater than or equal to
  /// 1, which occurs when @p min_theta equals @p max_theta.
  ///
  /// @param[in] num_phi_values The number of theta values at which depth
  /// measurements are taken. These measurements are evenly spread between
  /// @p min_phi and @p max_phi. This value must be greater than or equal to 1,
  /// which occurs when @p min_phi equals @p max_phi.
  ///
  /// @param[in] min_range The minimum sensing range.
  ///
  /// @param[in] max_range The maximum sensing range.
  ///
  DepthSensor(const std::string& name, const RigidBodyTree<double>& tree,
              const RigidBodyFrame<double>& frame, double min_theta,
              double max_theta, double min_phi, double max_phi,
              int num_theta_values, int num_phi_values, double min_range,
              double max_range);

  /// A constructor that initializes a DepthSensor using an
  /// DepthSensorSpecification.
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
  /// @param[in] specs The specifications of this sensor. A copy of these
  /// specifications is stored as a class member variable.
  ///
  DepthSensor(const std::string& name, const RigidBodyTree<double>& tree,
              const RigidBodyFrame<double>& frame,
              const DepthSensorSpecification& specs);

  // Non-copyable.
  /// @name Deleted Copy/Move Operations
  /// DepthSensor is neither copyable nor moveable.
  ///@{
  explicit DepthSensor(const DepthSensor&) = delete;
  DepthSensor& operator=(const DepthSensor&) = delete;
  ///@}

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
  /// This is equal to parameter `num_phi_values` that's passed into the
  /// constructor.
  int get_num_pixel_rows() const { return specification_.num_phi_values(); }

  /// Returns the number of pixel columns in the resulting depth sensor output.
  /// This is equal to parameter `num_theta_values` that's passed into the
  /// constructor.
  int get_num_pixel_cols() const { return specification_.num_theta_values(); }

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

  // System<double> overrides.

  /// Allocates the output port. See this class' description for details of this
  /// port.
  std::unique_ptr<SystemOutput<double>> AllocateOutput(
      const Context<double>& context) const override;

  friend std::ostream& operator<<(std::ostream& out,
                                  const DepthSensor& depth_sensor);

 protected:
  /// Outputs the depth information.
  void DoCalcOutput(const systems::Context<double>& context,
                    systems::SystemOutput<double>* output) const override;

 private:
  // The depth sensor will cast a ray with its start point at (0,0,0) in the
  // sensor's frame (as defined by RigidBodyTreeSensor::get_frame()). Its end,
  // in the sensor's frame, is computed by this method and stored in member
  // variable raycast_endpoints_ at the time of construction. raycast_endpoints_
  // is only computed once at construction since the end points are constant in
  // the sensor's frame.
  //
  // TODO(liang.fok): fix the documentation below -- it doesn't make sense why
  // it's focused on the yaw or pitch direction.
  //
  // The end points are computed by scanning in the yaw (pitch) direction
  // discretizing the yaw (pitch) range in num_pixel_cols (num_pixel_rows).
  // The final 3D end point then corresponds to a ray that starts at zero, with
  // length max_range, at the specific yaw (pitch) angle.
  void CacheRaycastEndpoints();

  const std::string name_;
  const RigidBodyTree<double>& tree_;
  const RigidBodyFrame<double> frame_;
  const DepthSensorSpecification specification_;
  int state_input_port_id_{};
  int state_output_port_id_{};

  // A cache of where a depth measurement ray endpoint would be if the maximum
  // range were achieved. This is cached to avoid repeated allocation.
  std::unique_ptr<Eigen::Matrix3Xd> raycast_endpoints_;
};

}  // namespace sensors
}  // namespace systems
}  // namespace drake
