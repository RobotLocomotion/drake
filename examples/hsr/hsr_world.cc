#include "drake/examples/hsr/hsr_world.h"

namespace drake {
namespace examples {
namespace hsr {

using drake::geometry::SceneGraph;
using drake::multibody::Frame;
using drake::multibody::MultibodyPlant;

template <typename T>
HsrWorld<T>::HsrWorld(const std::string& config_file)
    : config_file_(config_file),
      owned_plant_(std::make_unique<MultibodyPlant<T>>(1e-3)),
      owned_scene_graph_(std::make_unique<SceneGraph<T>>()) {
  // This class holds the unique_ptrs explicitly for plant and scene_graph
  // until Finalize() is called (when they are moved into the Diagram). Grab
  // the raw pointers, which should stay valid for the lifetime of the Diagram.
  scene_graph_ = owned_scene_graph_.get();
  scene_graph_->set_name("scene_graph");

  plant_ = owned_plant_.get();
  plant_->RegisterAsSourceForSceneGraph(scene_graph_);
  plant_->set_name("plant");

  this->set_name("hsr_world");

  // Load the models here. Something like
  // const auto config_params = LoadWorldConfigurationParameters(config_file_);

  // Parse urdfs to get the models from the configuration parameters.
  // SetupWorld(config_params);

  // This function will finalize the plant and all the ports.
  // Finalize();
}

template <typename T>
void HsrWorld<T>::RegisterRgbdSensor(
    const std::string& name, const Frame<T>& parent_frame,
    const math::RigidTransform<double>& X_PC,
    const geometry::render::CameraProperties& color_properties,
    const geometry::render::DepthCameraProperties& depth_properties,
    RobotParameters<T>* robot_parameters) {
  CameraParameters<T> param;
  param.location.parent_frame = &parent_frame;
  param.location.X_PC = X_PC;
  param.color_properties = color_properties;
  param.depth_properties = depth_properties;

  const auto res = robot_parameters->camera_parameters.insert({name, param});
  if (!res.second) {
    drake::log()->warn("The camera: " + name +
                       " already registered. Skip adding this one");
  }
}

template <typename T>
void HsrWorld<T>::RegisterImuSensor(const std::string& name,
                                    const Frame<T>& parent_frame,
                                    const math::RigidTransform<double>& X_PC,
                                    RobotParameters<T>* robot_parameters) {
  SensorLocationParameters<T> imu_location;
  imu_location.parent_frame = &parent_frame;
  imu_location.X_PC = X_PC;

  const auto res =
      robot_parameters->imu_parameters.insert({name, imu_location});
  if (!res.second) {
    drake::log()->warn("The imu: " + name +
                       " already registered. Skip adding this one");
  }
}

template <typename T>
void HsrWorld<T>::RegisterForceSensor(const std::string& name,
                                      const Frame<T>& parent_frame,
                                      const math::RigidTransform<double>& X_PC,
                                      RobotParameters<T>* robot_parameters) {
  SensorLocationParameters<T> force_sensor_location;
  force_sensor_location.parent_frame = &parent_frame;
  force_sensor_location.X_PC = X_PC;

  const auto res = robot_parameters->force_sensor_parameters.insert(
      {name, force_sensor_location});
  if (!res.second) {
    drake::log()->warn("The force sensor: " + name +
                       " already registered. Skip adding this one");
  }
}

}  // namespace hsr
}  // namespace examples
}  // namespace drake

template class drake::examples::hsr::HsrWorld<double>;
