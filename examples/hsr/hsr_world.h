#pragma once

#include <map>
#include <string>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/geometry/render/render_engine.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram.h"

namespace drake {
namespace examples {
namespace hsr {

/// A system that represents the complete HSR world environment, including
/// the robots and anything a user might want to load into the model such as
/// objects or sensors. All these Parameterss are stored in the configration
/// yaml file. For the proof of life purpose, only a simple table and a
/// bottle are added as the extra objects.
///
/// @system{HsrWorld,
///   @input_port{hsr_${name}_desired_state}
///   @output_port{hsr_${name}_commanded_position}
///   @output_port{hsr_${name}_measured_position}
///   @output_port{hsr_${name}_estimated_velocity}
///   @output_port{hsr_${name}_estimated_state}
///   @output_port{hsr_${name}_torque_external}
///   @output_port{hsr_${name}_generalized_force}
///   @output_port{hsr_${name}_actuation_commanded}
///   @output_port{pose_bundle}
///   @output_port{contact_results}
///   @output_port{plant_continuous_state}
///   @output_pott{geometry_poses}
///   Availablity of the following ports depends on the configuration.
///   @output_port{imu_${name}_status}
///   @output_port{force_sensor_${name}_status}
///   @output_port{camera_${name}_rgb_image}
///   @output_port{camera_${name}_depth_image}
///   @output_port{camera_${name}_label_image}
/// }
/// Note that, the exact name of the port will depend on the name of the items
/// since the world may contain more than one robot and more than one same
/// sensors.

template <typename T>
class HsrWorld : public systems::Diagram<T> {
 public:
  /// TODO(huihua) Maybe should move these parameters into a separate header.
  /// All these parameters should be able to be loaded from the configuration
  /// files.
  struct SensorLocationParameters {
    const multibody::Frame<T>* parent_frame{};
    math::RigidTransform<double> X_PC{math::RigidTransform<double>::Identity()};
  };

  struct CameraParameters {
    SensorLocationParameters location;
    geometry::render::CameraProperties color_properties{0, 0, 0, ""};
    geometry::render::DepthCameraProperties depth_properties{0, 0, 0, "", 0, 0};
  };

  struct JointParameters {
    explicit JointParameters(const std::string& joint_name)
        : name(joint_name) {}
    std::string name;
    double position_offset{0.0};
    double position_limit_lower{0.0};
    double position_limit_upper{0.0};
    double velocity_limit_lower{0.0};
    double velocity_limit_upper{0.0};
    double torque_limit_lower{0.0};
    double torque_limit_upper{0.0};
  };

  /// Parameters of the parts that assemble a robot.
  struct PartParameters {
    explicit PartParameters(const std::string& part_name) : name(part_name) {}
    std::string name;
    std::vector<JointParameters> joints_parameters;
  };

  /// Parameters of a robot that includes different moving parts and the
  /// sensors.
  struct RobotParameters {
    std::string name;
    std::map<std::string, CameraParameters> camera_Parameters;
    std::map<std::string, SensorLocationParameters> force_sensor_Parameters;
    std::map<std::string, SensorLocationParameters> imu_Parameters;
    std::map<std::string, PartParameters> parts_Parameters;
  };

  /// Contains all the necessary information about a loaded model instance.
  /// This information will be useful if extra care is needed for this instance.
  /// For example, the X_PC will be used to set the initial position if this
  /// instance has a floating base.
  struct ModelInstanceInfo {
    std::string model_name;
    std::string model_path;
    std::string parent_frame_name;
    std::string child_frame_name;
    multibody::ModelInstanceIndex index;
    drake::math::RigidTransform<T> X_PC{
        math::RigidTransform<double>::Identity()};
  };

  /// @param config_file path to the configuration file to load.
  HsrWorld(const std::string& config_file);

 private:
  const std::string default_renderer_name_ = "hsr_world_renderer";
  const std::string config_file_;

  // These are only valid until Finalize() is called.
  std::unique_ptr<drake::multibody::MultibodyPlant<T>> owned_plant_;
  std::unique_ptr<drake::geometry::SceneGraph<T>> owned_scene_graph_;

  // These are valid for the lifetime of this system.
  drake::multibody::MultibodyPlant<T>* plant_{};
  drake::geometry::SceneGraph<T>* scene_graph_{};

  std::map<std::string, RobotParameters> robots_Parameters_;

  std::map<std::string, ModelInstanceInfo> robots_instance_info_;
  std::map<std::string, ModelInstanceInfo> items_instance_info_;

  /// Create internal plants for the robots.
  using mbp_unique_ptr = std::unique_ptr<multibody::MultibodyPlant<T>>;
  std::vector<mbp_unique_ptr> owned_robot_plants_;
  /// The welded version of the same robot plant is created for the purpose of
  /// using inverse dynamics controller, which only works for fully actuated
  /// systems.
  std::vector<mbp_unique_ptr> owned_welded_robot_plants_;
};

}  // namespace hsr
}  // namespace examples
}  // namespace drake
