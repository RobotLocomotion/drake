#pragma once

#include <map>
#include <string>
#include <vector>

#include "drake/geometry/render/camera_properties.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace examples {
namespace hsr {

/// All these parameters should be able to be loaded from the configuration
/// files.
template <typename T>
struct SensorLocationParameters {
  const multibody::Frame<T>* parent_frame{};  ///< Frame where the sensor will
                                              ///< be "mounted" on.
  math::RigidTransform<T> X_PC;               ///< Offset of the mounting
                                              ///< position w.r.t the parent
                                              ///< frame.
};

/// Both the extrinsic and intrinsic parameters of a render camera.
template <typename T>
struct CameraParameters {
  SensorLocationParameters<T> location;
  geometry::render::CameraProperties color_properties{0, 0, 0, ""};
  geometry::render::DepthCameraProperties depth_properties{0, 0, 0, "", 0, 0};
};

/// A simple struct to store PID gains for one joint.
struct PidGains {
  double kp{0.0};
  double ki{0.0};
  double kd{0.0};
};

/// A struct stores the necessary parameters to control a single joint.
struct JointParameters {
  explicit JointParameters(const std::string& joint_name) : name(joint_name) {}
  std::string name;
  double position_offset{0.0};       ///< [rad] Offset from the nominal zero
                                     ///< position.
  double position_limit_lower{0.0};  ///< [rad]
  double position_limit_upper{0.0};  ///< [rad]
  double velocity_limit{0.0};        ///< [rad/s] Absolute value of the
                                     ///< velocity limit.
  double torque_limit{0.0};          ///< [Nm/rad] for revolute joint, [N/m] for
                                     ///< linear joint. Absolute value of the
                                     ///< torque limit.
  PidGains pid_gains;                ///< PID gains configured for the joint.
};

/// Parameters of the parts that assemble a robot. Parts are defined as the
/// major components of a robot. To be more specific, a part could mean the
/// torso, arm, head, base/chassis, or grippers. Each part may have a different
/// controller strategy. We are only interested in the actuated joints inside
/// a part.
struct PartParameters {
  explicit PartParameters(const std::string& part_name) : name(part_name) {}
  std::string name;
  std::vector<JointParameters> joints_parameters;
};

/// Parameters of a robot that includes different parts and the sensors.
template <typename T>
struct RobotParameters {
  std::string name;
  std::map<std::string, CameraParameters<T>> camera_parameters;
  std::map<std::string, SensorLocationParameters<T>> force_sensor_parameters;
  std::map<std::string, SensorLocationParameters<T>> imu_parameters;
  std::map<std::string, PartParameters> parts_parameters;
};

}  // namespace hsr
}  // namespace examples
}  // namespace drake
