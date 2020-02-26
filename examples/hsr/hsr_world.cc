#include "drake/examples/hsr/hsr_world.h"

#include <unordered_set>
#include <utility>

#include "drake/common/find_resource.h"
#include "drake/examples/hsr/controllers/main_controller.h"
#include "drake/examples/hsr/parameters/parameters.h"
#include "drake/geometry/render/render_engine_vtk_factory.h"
#include "drake/multibody/benchmarks/inclined_plane/inclined_plane_plant.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/primitives/demultiplexer.h"
#include "drake/systems/primitives/pass_through.h"

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
using drake::multibody::ModelInstanceIndex;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::Diagram;
using drake::systems::State;
using Eigen::VectorXd;

namespace {
/// Converts the generalized force output of the ID controller (internally using
/// a control plant with only the gripper) to the generalized force input for
/// the full simulation plant (containing gripper and object).
class RobotToPlantForceConverter final : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RobotToPlantForceConverter);
  RobotToPlantForceConverter(const MultibodyPlant<double>& plant,
                             ModelInstanceIndex robot_instance)
      : plant_(plant), robot_instance_(robot_instance) {
    this->DeclareVectorInputPort(
        "input", BasicVector<double>(plant_.num_velocities(robot_instance_)));
    this->DeclareVectorOutputPort("output",
                                  BasicVector<double>(plant_.num_velocities()),
                                  &RobotToPlantForceConverter::remap_output);
  }

  void remap_output(const Context<double>& context,
                    BasicVector<double>* output_vector) const {
    auto output_value = output_vector->get_mutable_value();
    auto input_value = get_input_port(0).Eval(context);

    output_value.setZero();
    plant_.SetVelocitiesInArray(robot_instance_, input_value, &output_value);
  }

 private:
  const MultibodyPlant<double>& plant_;
  const ModelInstanceIndex robot_instance_;
};
}  // namespace

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

  // Parse urdfs to get the models from the configuration parameters.
  const std::vector<ModelInstanceInfo<T>> loaded_models = LoadModelsFromUrdfs();
  SetupWorld(loaded_models);

  // This function will finalize the plant and all the ports.
  Finalize();
}

template <typename T>
const std::vector<ModelInstanceInfo<T>>
HsrWorld<T>::LoadModelsFromConfigurationFile() {
  std::vector<ModelInstanceInfo<T>> added_models;
  return added_models;
}

// Add default HSR.
template <typename T>
const ModelInstanceInfo<T> HsrWorld<T>::AddDefaultHsr() {
  const std::string model_path = FindResourceOrThrow(
      "drake/examples/hsr/models/urdfs/hsrb4s_fix_free_joints.urdf");
  const std::string model_name = "hsr";

  Parser parser(plant_);
  const multibody::ModelInstanceIndex model_instance_index =
      parser.AddModelFromFile(model_path, model_name);

  ModelInstanceInfo<T> robot_instace_info;
  robot_instace_info.model_name = model_name;
  robot_instace_info.model_path = model_path;
  robot_instace_info.parent_frame_name = "world_frame";
  robot_instace_info.child_frame_name = "base_footprint";
  robot_instace_info.index = model_instance_index;
  robot_instace_info.X_PC = RigidTransform<double>::Identity();

  return robot_instace_info;
}

template <typename T>
const std::vector<ModelInstanceInfo<T>> HsrWorld<T>::LoadModelsFromUrdfs() {
  std::vector<ModelInstanceInfo<T>> added_models;

  added_models.push_back(this->AddDefaultHsr());

  return added_models;
}

template <typename T>
void HsrWorld<T>::SetDefaultState(const Context<T>& context,
                                  State<T>* state) const {
  // Call the base class method, to initialize all systems in this diagram.
  Diagram<T>::SetDefaultState(context, state);

  // Use initial position of the HSR robot.
  const auto& default_hsr_info = robots_instance_info_.find("hsr");
  // HsrWorld should at least have one default HSR robot.
  DRAKE_DEMAND(robots_instance_info_.find("hsr") !=
               robots_instance_info_.end());

  const auto& hsr_instance = default_hsr_info->second.index;
  SetModelPositionState(context, hsr_instance,
                        GetModelPositionState(context, hsr_instance), state);
  SetModelVelocityState(context, hsr_instance,
                        GetModelVelocityState(context, hsr_instance), state);
}

template <typename T>
void HsrWorld<T>::SetupWorld(
    const std::vector<ModelInstanceInfo<T>>& added_models) {
  const auto& sim_params = hsr_sim_flags();
  // Add a ground plane.
  {
    const multibody::CoulombFriction<double> coef_friction_inclined_plane(
        sim_params.inclined_plane_coef_static_friction,
        sim_params.inclined_plane_coef_kinetic_friction);
    multibody::benchmarks::inclined_plane::AddInclinedPlaneAndGravityToPlant(
        sim_params.gravity, 0.0, std::nullopt, coef_friction_inclined_plane,
        plant_);
  }

  // Use model directives to load the robot and the world.
  {
    // Parse the added models to different catagories.
    for (const auto& model_info : added_models) {
      if (model_info.model_name.find("hsr") != std::string::npos) {
        robots_instance_info_.insert({model_info.model_name, model_info});
        continue;
      } else {
        items_instance_info_.insert({model_info.model_name, model_info});
        continue;
      }
    }
  }
  // HsrWorld should at least have one default HSR robot.
  DRAKE_DEMAND(robots_instance_info_.find("hsr") !=
               robots_instance_info_.end());
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
VectorX<T> HsrWorld<T>::GetModelPositionState(
    const Context<T>& context, const ModelInstanceIndex& model_index) const {
  const auto& plant_context = this->GetSubsystemContext(*plant_, context);
  return plant_->GetPositions(plant_context, model_index);
}

template <typename T>
VectorX<T> HsrWorld<T>::GetModelVelocityState(
    const Context<T>& context, const ModelInstanceIndex& model_index) const {
  const auto& plant_context = this->GetSubsystemContext(*plant_, context);
  return plant_->GetVelocities(plant_context, model_index);
}

template <typename T>
void HsrWorld<T>::SetModelPositionState(
    const Context<T>& context, const ModelInstanceIndex& model_index,
    const Eigen::Ref<const drake::VectorX<T>>& q, State<T>* state) const {
  DRAKE_DEMAND(state != nullptr);
  const int num_model_positions = plant_->num_positions(model_index);
  DRAKE_DEMAND(q.size() == num_model_positions);
  auto& plant_context = this->GetSubsystemContext(*plant_, context);
  auto& plant_state = this->GetMutableSubsystemState(*plant_, state);
  plant_->SetPositions(plant_context, &plant_state, model_index, q);
}

template <typename T>
void HsrWorld<T>::SetModelVelocityState(
    const Context<T>& context, const ModelInstanceIndex& model_index,
    const Eigen::Ref<const drake::VectorX<T>>& v, State<T>* state) const {
  DRAKE_DEMAND(state != nullptr);
  const int num_model_velocities = plant_->num_velocities(model_index);
  DRAKE_DEMAND(v.size() == num_model_velocities);
  auto& plant_context = this->GetSubsystemContext(*plant_, context);
  auto& plant_state = this->GetMutableSubsystemState(*plant_, state);
  plant_->SetVelocities(plant_context, &plant_state, model_index, v);
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

  const auto& hsr_instance = robots_instance_info_["hsr"].index;
  const int num_hsr_positions = plant_->num_positions(hsr_instance);
  const int num_hsr_velocities = plant_->num_velocities(hsr_instance);

  // Export HSR "state" outputs.
  {
    auto demux = builder.template AddSystem<systems::Demultiplexer>(
        std::vector<int>{num_hsr_positions, num_hsr_velocities});
    builder.Connect(plant_->get_state_output_port(hsr_instance),
                    demux->get_input_port(0));
    builder.ExportOutput(demux->get_output_port(0), "hsr_position_measured");
    builder.ExportOutput(demux->get_output_port(1), "hsr_velocity_estimated");
    builder.ExportOutput(plant_->get_state_output_port(hsr_instance),
                         "hsr_state_estimated");
  }

  // Connect the states with controllers.
  // TODO(huihua) We currently assume that there is only one default hsr robot.
  {
    const auto& hsr_owned_robots_plant = owned_robots_plant_.find("hsr");
    auto hsr_main_controller =
        builder.template AddSystem<controller::MainController>(
            *(hsr_owned_robots_plant->second.float_plant),
            *(hsr_owned_robots_plant->second.welded_plant),
            robots_Parameters_["hsr"]);

    builder.ExportInput(hsr_main_controller->get_desired_state_input_port(),
                        "hsr_desired_state");

    builder.Connect(plant_->get_state_output_port(hsr_instance),
                    hsr_main_controller->get_estimated_state_input_port());

    // The hsr main controller internally uses the "hsr plant",
    // which contains the hsr model *only* (i.e., no object). Therefore,
    // its output must be re-mapped to the input of the full "simulation
    // plant", which contains both hsr and other objects. The system
    // hsrToSimPlantForceConverter fills this role.
    // Generalized force is calculated for the upper body.
    auto generalized_force_map =
        builder.template AddSystem<RobotToPlantForceConverter>(*plant_,
                                                               hsr_instance);

    builder.Connect(hsr_main_controller->get_generalized_force_output_port(),
                    generalized_force_map->get_input_port(0));
    builder.Connect(generalized_force_map->get_output_port(0),
                    plant_->get_applied_generalized_force_input_port());

    builder.ExportOutput(
        hsr_main_controller->get_generalized_force_output_port(),
        "hsr_generalized_force");

    builder.Connect(hsr_main_controller->get_actuation_output_port(),
                    plant_->get_actuation_input_port(hsr_instance));

    builder.ExportOutput(hsr_main_controller->get_actuation_output_port(),
                         "hsr_actuation_commanded");
  }

  builder.ExportOutput(scene_graph_->get_pose_bundle_output_port(),
                       "pose_bundle");
  builder.ExportOutput(plant_->get_contact_results_output_port(),
                       "contact_results");
  builder.ExportOutput(plant_->get_geometry_poses_output_port(),
                       "geometry_poses");

  builder.BuildInto(this);
}

}  // namespace hsr
}  // namespace examples
}  // namespace drake

template class drake::examples::hsr::HsrWorld<double>;
