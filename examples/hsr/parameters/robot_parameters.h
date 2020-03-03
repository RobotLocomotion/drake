#pragma once

#include <map>
#include <string>
#include <vector>

#include "drake/common/name_value.h"
#include "drake/geometry/render/camera_properties.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/tree/frame.h"

namespace drake {
namespace examples {
namespace hsr {
namespace parameters {

/// All these parameters should be able to be loaded from the configuration
/// files.
template <typename T>
struct SensorLocationParameters {
  std::string parent_frame_name;  ///< Name of the frame where the sensor
                                  ///< will be "mounted" on.
  math::RigidTransform<T> X_PC;   ///< Offset of the mounting position
                                  ///< w.r.t the parent frame.
};

/// Both the extrinsic and intrinsic parameters of a render camera.
template <typename T>
struct CameraParameters {
  SensorLocationParameters<T> location;
  geometry::render::CameraProperties color_properties{0, 0, 0, ""};
  geometry::render::DepthCameraProperties depth_properties{0, 0, 0, "", 0, 0};
};

/// A simple struct to store PID gains for one joint. Serialize the members so
/// that the struct can be loaded from yaml files using the YamlReadArchive.
/// Please refer to "examples/hsr/parameters/robot_parameters_loader.h" for
/// details.
struct PidGains {
  double kp{0.0};
  double ki{0.0};
  double kd{0.0};

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(kp));
    a->Visit(DRAKE_NVP(ki));
    a->Visit(DRAKE_NVP(kd));
  }
};

/// A struct stores the necessary parameters to control a single joint.
/// Serialize the members so that the struct can be loaded from yaml files
/// using the YamlReadArchive.
struct JointParameters {
  explicit JointParameters(const std::string& joint_name = "")
      : name(joint_name) {}
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
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(name));
    a->Visit(DRAKE_NVP(position_offset));
    a->Visit(DRAKE_NVP(position_limit_lower));
    a->Visit(DRAKE_NVP(position_limit_upper));
    a->Visit(DRAKE_NVP(velocity_limit));
    a->Visit(DRAKE_NVP(torque_limit));
    a->Visit(DRAKE_NVP(pid_gains));
  }
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
  std::map<std::string, CameraParameters<T>> cameras_parameters;
  std::map<std::string, SensorLocationParameters<T>> force_sensors_parameters;
  std::map<std::string, SensorLocationParameters<T>> imus_parameters;
  std::map<std::string, PartParameters> parts_parameters;
};

}  // namespace parameters
}  // namespace hsr
}  // namespace examples
}  // namespace drake
