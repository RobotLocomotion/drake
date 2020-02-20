#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/examples/hsr/common/model_instance_info.h"
#include "drake/examples/hsr/common/robot_parameters.h"
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
///   @input_port{hsr_[name]_desired_state}
///   @output_port{hsr_[name]_commanded_position}
///   @output_port{hsr_[name]_measured_position}
///   @output_port{hsr_[name]_estimated_velocity}
///   @output_port{hsr_[name]_estimated_state}
///   @output_port{hsr_[name]_torque_external}
///   @output_port{hsr_[name]_generalized_force}
///   @output_port{hsr_[name]_actuation_commanded}
///   @output_port{pose_bundle}
///   @output_port{contact_results}
///   @output_port{plant_continuous_state}
///   @output_pott{geometry_poses}
///   Availablity of the following ports depends on the configuration.
///   @output_port{[name]_imu_status}
///   @output_port{[name]_force_sensor_status}
///   @output_port{[name]_camera_rgb_image}
///   @output_port{[name]_camera_depth_image}
///   @output_port{[name]_camera_label_image}
/// }
/// Note that, the exact name of the port will depend on the name of the items
/// since the world may contain more than one robot and more than one same
/// sensors.

template <typename T>
class HsrWorld : public systems::Diagram<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(HsrWorld);
  /// @param config_file path to the configuration file to load.
  explicit HsrWorld(const std::string& config_file);

 private:
  /// Registers a RGBD sensor into the parameters of a robot. Must be called
  /// before Finalize().
  /// @param name Name for the camera.
  /// @param parent_frame The parent frame (frame P). The body that
  ///   `parent_frame` is attached to must have a corresponding
  ///   geometry::FrameId. Otherwise, an exception will be thrown in Finalize().
  /// @param X_PC Transformation between frame P and the camera body.
  ///   see systems::sensors::RgbdCamera for descriptions about how the
  ///   camera body, RGB, and depth image frames are related.
  /// @param color_properties Properties for the color image of the camera.
  /// @param depth_properties Properties for the depth image of the camera.
  /// @param robot_parameters The target robot where the RGBD sensor will be
  ///   registered.
  void RegisterRgbdSensor(
      const std::string& name, const multibody::Frame<T>& parent_frame,
      const math::RigidTransform<double>& X_PC,
      const geometry::render::CameraProperties& color_properties,
      const geometry::render::DepthCameraProperties& depth_properties,
      RobotParameters<T>* robot_parameters);

  /// Registers an IMU sensor. Must be called before Finalize().
  /// @param name    Name of the joint where the IMU sensor is mounted. It
  ///   must be a valid joint name of the multibody plant. Otherwise, the
  ///   system will throw. The name should also be unique compared to the
  ///   already registered IMU sensors. If not, a warning will be
  ///   shown and the IMU sensor won't be added.
  /// @param parent_frame The parent frame (frame P). The body that
  ///   `parent_frame` is attached to must have a corresponding
  ///   geometry::FrameId. Otherwise, an exception will be thrown in Finalize().
  /// @param X_PC Transformation between parent frame P and the IMU sensor.
  /// @param robot_parameters The target robot where the RGBD sensor will be
  ///   registered.
  /// N.B. Since the IMU emulator has not been implemented yet, a dummy name
  /// has been provided. It does not correspond to any actual joint name.
  /// For every successfully registered IMU sensor joint,  a output port
  /// with name `[name]+_imu_status` will be created.
  void RegisterImuSensor(const std::string& name,
                         const multibody::Frame<T>& parent_frame,
                         const math::RigidTransform<double>& X_PC,
                         RobotParameters<T>* robot_parameters);

  /// Registers a force sensor. Must be called before Finalize().
  /// @param name    Name of the joint where the force sensor is mounted. It
  ///   must be a valid joint name of the multibody plant. Otherwise, the
  ///   system will throw. The name should also be unique compared to the
  ///   already registered joint force sensors. If not, a warning will be
  ///   shown and the force sensor won't be added.
  /// @param parent_frame The parent frame (frame P). The body that
  ///   `parent_frame` is attached to must have a corresponding
  ///   geometry::FrameId. Otherwise, an exception will be thrown in Finalize().
  /// @param X_PC Transformation between parent frame P and the force sensor.
  /// @param robot_parameters The target robot where the RGBD sensor will be
  ///   registered.
  /// For every successfully registered force sensor joint,  a output port
  /// with name `[name]+_force_sensor_status` will be created.
  void RegisterForceSensor(const std::string& name,
                           const multibody::Frame<T>& parent_frame,
                           const math::RigidTransform<double>& X_PC,
                           RobotParameters<T>* robot_parameters);

  const std::string default_renderer_name_ = "hsr_world_renderer";
  const std::string config_file_;

  // These are only valid until Finalize() is called.
  std::unique_ptr<drake::multibody::MultibodyPlant<T>> owned_plant_;
  std::unique_ptr<drake::geometry::SceneGraph<T>> owned_scene_graph_;

  // These are valid for the lifetime of this system.
  drake::multibody::MultibodyPlant<T>* plant_{};
  drake::geometry::SceneGraph<T>* scene_graph_{};

  std::map<std::string, RobotParameters<T>> robots_Parameters_;

  std::map<std::string, ModelInstanceInfo<T>> robots_instance_info_;
  std::map<std::string, ModelInstanceInfo<T>> items_instance_info_;

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
