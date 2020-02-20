#include "drake/examples/hsr/hsr_world.h"

#include <unordered_set>
#include <utility>

#include "drake/examples/hsr/parameters.h"
#include "drake/geometry/render/render_engine_vtk_factory.h"
#include "drake/multibody/parsing/parser.h"
namespace drake {
namespace examples {
namespace hsr {

using drake::geometry::SceneGraph;
using drake::geometry::render::MakeRenderEngineVtk;
using drake::geometry::render::RenderEngineVtkParams;
using drake::math::RigidTransform;
using drake::math::RigidTransformd;
using drake::multibody::BodyIndex;
using drake::multibody::Frame;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;

template <typename T>
HsrWorld<T>::HsrWorld(const std::string& config_file)
    : config_file_(config_file),
      owned_plant_(
          std::make_unique<MultibodyPlant<T>>(hsr_sim_flags().time_step)),
      owned_scene_graph_(std::make_unique<SceneGraph<T>>()) {
  // This class holds the unique_ptrs explicitly for plant and scene_graph
  // until Finalize() is called (when they are moved into the Diagram). Grab
  // the raw pointers, which should stay valid for the lifetime of the Diagram.
  scene_graph_ = owned_scene_graph_.get();
  scene_graph_->set_name("scene_graph");
  // Setup the render engine. Choose to use the default for now.
  scene_graph_->AddRenderer("hsr_world_renderer",
                            MakeRenderEngineVtk(RenderEngineVtkParams()));

  plant_ = owned_plant_.get();
  plant_->RegisterAsSourceForSceneGraph(scene_graph_);
  plant_->set_name("plant");

  this->set_name("hsr_world");

  // Load the models here. Something like
  // const auto config_params = LoadWorldConfigurationParameters(config_file_);

  // Parse urdfs to get the models from the configuration parameters.
  // SetupWorld(config_params);

  // This function will finalize the plant and all the ports.
  Finalize();
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

template <typename T>
void HsrWorld<T>::MakeRobotControlPlants() {
  // Build the robot plants for controller purpose. It contains both the
  // floating robot model and the same model but with the base welded to
  // the ground.
  for (const auto& robot_instance_info : robots_instance_info_) {
    OwnedRobotControllerPlant owned_plants(hsr_sim_flags().time_step);

    Parser(owned_plants.float_plant.get())
        .AddModelFromFile(robot_instance_info.second.model_path);
    owned_plants.float_plant->set_name(robot_instance_info.first);
    owned_plants.float_plant->Finalize();

    // Create the welded version for inverse dynamic controller.
    const auto welded_robot_model =
        Parser(owned_plants.welded_plant.get())
            .AddModelFromFile(robot_instance_info.second.model_path);

    // The welded plant is only used for the inverse dynamics controller
    // calculation purpose. Here we assume the robot only has one floating
    // body, which should be true.
    const std::unordered_set<BodyIndex> floating_base_indexes =
        owned_plants.float_plant->GetFloatingBaseBodies();
    DRAKE_DEMAND(floating_base_indexes.size() == 1);

    owned_plants.welded_plant->WeldFrames(
        owned_plants.welded_plant->world_frame(),
        owned_plants.welded_plant->GetFrameByName(
            owned_plants.float_plant->get_body(*(floating_base_indexes.begin()))
                .name(),
            welded_robot_model),
        robot_instance_info.second.X_PC);

    owned_plants.welded_plant->set_name("welded_" + robot_instance_info.first);
    owned_plants.welded_plant->Finalize();

    owned_robots_plant_.insert(
        {robot_instance_info.first, std::move(owned_plants)});
  }
}

template <typename T>
void HsrWorld<T>::Finalize() {
  MakeRobotControlPlants();

  // Note: This deferred diagram construction method/workflow exists because we
  //   - cannot finalize plant until all of the objects are added, and
  //   - cannot wire up the diagram until we have finalized the plant.
  plant_->Finalize();

  const auto& sim_params = hsr_sim_flags();
  plant_->set_penetration_allowance(sim_params.penetration_allowance);
  plant_->set_stiction_tolerance(sim_params.v_stiction_tolerance);

  systems::DiagramBuilder<T> builder;
  builder.AddSystem(std::move(owned_plant_));
  builder.AddSystem(std::move(owned_scene_graph_));

  builder.Connect(
      plant_->get_geometry_poses_output_port(),
      scene_graph_->get_source_pose_port(plant_->get_source_id().value()));
  builder.Connect(scene_graph_->get_query_output_port(),
                  plant_->get_geometry_query_input_port());
}

}  // namespace hsr
}  // namespace examples
}  // namespace drake

template class drake::examples::hsr::HsrWorld<double>;
