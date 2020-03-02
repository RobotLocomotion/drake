#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/examples/hsr/common/model_instance_info.h"
#include "drake/examples/hsr/parameters/robot_parameters.h"
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
///   @input_port{[name]_desired_state}
///   @output_port{[name]_commanded_position}
///   @output_port{[name]_measured_position}
///   @output_port{[name]_estimated_velocity}
///   @output_port{[name]_estimated_state}
///   @output_port{[name]_torque_external}
///   @output_port{[name]_generalized_force}
///   @output_port{[name]_actuation_commanded}
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
  /// Internal plants created for control purposes. The welded version of the
  /// same robot plant is created for the purpose of using the inverse
  /// dynamics controller (IDC), which only works for fully actuated systems.
  /// To be more specific, the Drake IDC only supports fully actuated systems
  /// for now. Robots with a mobile base are underactuated. We have to weld
  /// the base to the ground or someplace to get a fully actuated system,
  /// i.e., each moving joint should have one and only actuator associated
  /// to it. Welding the base of a mobile robot to the ground does not
  /// necessarily yield a fully actuated system though. However, if the
  /// users want to use the IDC, this assumption has to be true. Otherwise,
  /// a vanilla PID controller or PID with gravity compensation controller
  /// can be used.
  ///
  /// The floating base robot (or the original robot plant) is necessary to map
  /// the states of the welded plant back to the states of the original plant.
  /// Then the states can be again easily inserted back to the state of the
  /// full `hsr_world` system. The original floating robot plant can also be
  /// useful for many other situations such as reading and setting
  /// joint commands, etc.
  struct OwnedRobotControllerPlant {
    using mbp = multibody::MultibodyPlant<T>;
    explicit OwnedRobotControllerPlant(const double time_step)
        : float_plant(std::make_unique<mbp>(time_step)),
          welded_plant(std::make_unique<mbp>(time_step)) {}
    std::unique_ptr<mbp> float_plant;
    std::unique_ptr<mbp> welded_plant;
  };

  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(HsrWorld);
  /// @param config_file path to the configuration file to load.
  explicit HsrWorld(const std::string& config_file);

  /// Sets the default State using the Diagram's default state but override
  /// free body positions with data from the model directive yaml files and the
  /// default joint positions and velocities of the HSR robot from
  /// GetModelPositionState() and GetModelVelocityState().
  /// @param context A const reference to the HsrWorld context.
  /// @param state A pointer to the State of the HsrWorld system.
  /// @pre `state` must be the systems::State<T> object contained in `context`.
  void SetDefaultState(const systems::Context<T>& context,
                       systems::State<T>* state) const override;

  /// Users *must* call Finalize() after making any additions to the
  /// multibody plant and before using this class in the Systems framework.
  /// This should be called exactly once.
  ///
  /// @see multibody::MultibodyPlant<T>::Finalize()
  void Finalize();

  /// Return a reference to the floating base hsr plant used for the internal
  /// controller calculation purpose. If the given robot (by name) does not
  /// exist, it will throw.
  const multibody::MultibodyPlant<T>& get_hsr_plant(
      const std::string& hsr_name) const {
    const auto& hsr_owned_plant = owned_robots_plants_.find(hsr_name);
    DRAKE_DEMAND(hsr_owned_plant != owned_robots_plants_.end());
    return *(hsr_owned_plant->second.float_plant);
  }

  /// Returns a reference to the main plant responsible for the dynamics of
  /// the robot and the environment.  This can be used to, e.g., add
  /// additional elements into the world before calling Finalize().
  const multibody::MultibodyPlant<T>& get_multibody_plant() const {
    return *plant_;
  }

  /// Returns a mutable reference to the main plant responsible for the
  /// dynamics of the robot and the environment.  This can be used to, e.g.,
  /// add additional elements into the world before calling Finalize().
  multibody::MultibodyPlant<T>& get_mutable_multibody_plant() {
    return *plant_;
  }

  /// Returns a reference to the SceneGraph responsible for all of the geometry
  /// for the robot and the environment.  This can be used to, e.g., add
  /// additional elements into the world before calling Finalize().
  const geometry::SceneGraph<T>& get_scene_graph() const {
    return *scene_graph_;
  }

  /// Returns a mutable reference to the SceneGraph responsible for all of the
  /// geometry for the robot and the environment.  This can be used to, e.g.,
  /// add additional elements into the world before calling Finalize().
  geometry::SceneGraph<T>& get_mutable_scene_graph() { return *scene_graph_; }

  VectorX<T> GetModelPositionState(
      const systems::Context<T>& context,
      const multibody::ModelInstanceIndex& model_index) const;

  VectorX<T> GetModelVelocityState(
      const systems::Context<T>& context,
      const multibody::ModelInstanceIndex& model_index) const;

  /// Set the position state of a model that has at least one free moving joint.
  /// For example, the model could be a robot or a dishwasher.
  /// @param context A const reference to the HsrWorld context.
  /// @param model_index Const reference of the interested model instance index.
  /// @param q The target position state of the corresponding model to be set.
  /// @param state The full state of the robot world, must be the
  /// systems::State<T> object contained in `context`.
  void SetModelPositionState(const systems::Context<T>& context,
                             const multibody::ModelInstanceIndex& model_index,
                             const Eigen::Ref<const VectorX<T>>& q,
                             systems::State<T>* state) const;

  /// Set the velocity state of a model that has at least one free moving joint.
  /// For example, the model could be a robot or a dishwasher.
  /// @param context A const reference to the HsrWorld context.
  /// @param model_index Const reference of the interested model instance index.
  /// @param v The target velocity state of the corresponding model to be set.
  /// @param state The full state of the HsrWorld, must be the
  /// systems::State<T> object contained in `context`.
  void SetModelVelocityState(const systems::Context<T>& context,
                             const multibody::ModelInstanceIndex& model_index,
                             const Eigen::Ref<const VectorX<T>>& v,
                             systems::State<T>* state) const;

 private:
  /// Load all the models from the given configuration file. The return is a
  /// vector of model instance information.
  /// N.B. Anzu repo already has a nice implementation of loading models from
  /// yaml files. It's possible that Drake will have a taste of that feature
  /// soon. Before that, we will use the simple method which simply loads
  /// urdfs one by one.
  /// TODO(huihua) Once the model directive feature is in Drake, finish the
  /// implementation of this function.
  const std::vector<hsr::common::ModelInstanceInfo<T>>
  LoadModelsFromConfigurationFile() const;

  const std::vector<hsr::common::ModelInstanceInfo<T>> LoadModelsFromUrdfs()
      const;

  /// Load the robot parameters with a given @p`robot_name`.
  /// @throw std::exception if the robot parameter file of the given robot name
  /// does not exists.
  const hsr::parameters::RobotParameters<T> LoadRobotParameters(
      const std::string& robot_name) const;

  const hsr::common::ModelInstanceInfo<T> AddDefaultHsr() const;

  /// Post processing the loaded models such as setup the initial position of
  /// items
  void SetupWorld(
      const std::vector<hsr::common::ModelInstanceInfo<T>>& loaded_models);

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
      hsr::parameters::RobotParameters<T>* robot_parameters);

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
                         hsr::parameters::RobotParameters<T>* robot_parameters);

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
  void RegisterForceSensor(
      const std::string& name, const multibody::Frame<T>& parent_frame,
      const math::RigidTransform<double>& X_PC,
      hsr::parameters::RobotParameters<T>* robot_parameters);

  // Create two versions of plants for the controller purpose. One model with
  // floating base and one model with welded base. Assumes robots_instance_info_
  // has already being populated. Should only be called from Finalize().
  void MakeRobotControlPlants();

  const std::string config_file_;

  // These are only valid until Finalize() is called.
  std::unique_ptr<multibody::MultibodyPlant<T>> owned_plant_;
  std::unique_ptr<geometry::SceneGraph<T>> owned_scene_graph_;

  // These are valid for the lifetime of this system.
  multibody::MultibodyPlant<T>* plant_{};
  geometry::SceneGraph<T>* scene_graph_{};

  std::map<std::string, hsr::parameters::RobotParameters<T>> robots_parameters_;

  std::map<std::string, hsr::common::ModelInstanceInfo<T>>
      robots_instance_info_;
  std::map<std::string, hsr::common::ModelInstanceInfo<T>> items_instance_info_;

  /// Create internal plants for the robots.
  std::map<std::string, OwnedRobotControllerPlant> owned_robots_plants_;
};

}  // namespace hsr
}  // namespace examples
}  // namespace drake
