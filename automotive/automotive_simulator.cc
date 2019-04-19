#include "drake/automotive/automotive_simulator.h"

#include <algorithm>
#include <utility>

#include "drake/automotive/driving_command_mux.h"
#include "drake/automotive/gen/driving_command.h"
#include "drake/automotive/idm_controller.h"
#include "drake/automotive/maliput/api/junction.h"
#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/segment.h"
#include "drake/automotive/maliput/utility/generate_obj.h"
#include "drake/common/drake_throw.h"
#include "drake/common/find_resource.h"
#include "drake/common/temp_directory.h"
#include "drake/common/text_logging.h"
#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_driving_command_t.hpp"
#include "drake/lcmt_maliput_railcar_state_t.hpp"
#include "drake/lcmt_simple_car_state_t.hpp"
#include "drake/lcmt_viewer_draw.hpp"
#include "drake/math/rigid_transform.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/system.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/primitives/constant_value_source.h"
#include "drake/systems/rendering/render_pose_to_geometry_pose.h"

namespace drake {

using Eigen::Isometry3d;

using maliput::api::Lane;
using maliput::api::LaneEnd;
using maliput::api::LaneId;
using maliput::api::RoadGeometry;
using maliput::api::RoadGeometryId;
using systems::lcm::LcmInterfaceSystem;
using systems::lcm::LcmPublisherSystem;
using systems::lcm::LcmSubscriberSystem;
using systems::OutputPort;
using systems::rendering::PoseBundle;
using systems::RungeKutta2Integrator;
using systems::System;
using systems::SystemOutput;

namespace automotive {
namespace {

// Decodes Value<lcmt_driving_command_t> into DrivingCommand<double>.
class DrivingCommandDecoder final : public systems::LeafSystem<double> {
 public:
  DrivingCommandDecoder() {
    this->DeclareAbstractInputPort(
        systems::kUseDefaultName, Value<lcmt_driving_command_t>{});
    this->DeclareVectorOutputPort(
        systems::kUseDefaultName, DrivingCommand<double>{},
        &DrivingCommandDecoder::Calc);
  }
 private:
  void Calc(const systems::Context<double>& context,
            DrivingCommand<double>* output) const {
    const auto* const input =
        this->template EvalInputValue<drake::lcmt_driving_command_t>(
            context, 0);
    DRAKE_DEMAND(input != nullptr);
    output->set_steering_angle(input->steering_angle);
    output->set_acceleration(input->acceleration);
  }
};

// Encodes SimpleCarState<double> into Value<lcmt_simple_car_state_t>.
class SimpleCarStateEncoder final : public systems::LeafSystem<double> {
 public:
  SimpleCarStateEncoder() {
    this->DeclareVectorInputPort(
        systems::kUseDefaultName, SimpleCarState<double>{});
    this->DeclareAbstractOutputPort(
        systems::kUseDefaultName, lcmt_simple_car_state_t{},
        &SimpleCarStateEncoder::Calc);
  }
 private:
  void Calc(const systems::Context<double>& context,
            lcmt_simple_car_state_t* output) const {
    const auto* input =
        this->template EvalVectorInput<SimpleCarState>(context, 0);
    DRAKE_DEMAND(input != nullptr);
    *output = {};
    output->x = input->x();
    output->y = input->y();
    output->heading = input->heading();
    output->velocity = input->velocity();
  }
};

// Encodes MaliputRailcarState<double> into Value<lcmt_maliput_railcar_state_t>.
class MaliputRailcarStateEncoder final : public systems::LeafSystem<double> {
 public:
  MaliputRailcarStateEncoder() {
    this->DeclareVectorInputPort(
        systems::kUseDefaultName, MaliputRailcarState<double>{});
    this->DeclareAbstractOutputPort(
        systems::kUseDefaultName, lcmt_maliput_railcar_state_t{},
        &MaliputRailcarStateEncoder::Calc);
  }
 private:
  void Calc(const systems::Context<double>& context,
            lcmt_maliput_railcar_state_t* output) const {
    const auto* input =
        this->template EvalVectorInput<MaliputRailcarState>(context, 0);
    DRAKE_DEMAND(input != nullptr);
    *output = {};
    output->s = input->s();
    output->speed = input->speed();
  }
};

}  // namespace

template <typename T>
AutomotiveSimulator<T>::AutomotiveSimulator()
    : AutomotiveSimulator(std::make_unique<lcm::DrakeLcm>()) {}

template <typename T>
AutomotiveSimulator<T>::AutomotiveSimulator(
    std::unique_ptr<lcm::DrakeLcmInterface> lcm)
    : owned_lcm_(std::move(lcm)) {
  aggregator_ =
      builder_->template AddSystem<systems::rendering::PoseAggregator<T>>();
  aggregator_->set_name("pose_aggregator");

  if (owned_lcm_) {
    lcm_ = builder_->template AddSystem<LcmInterfaceSystem>(owned_lcm_.get());
    scene_graph_ = builder_->template AddSystem<geometry::SceneGraph>();
    geometry::ConnectDrakeVisualizer(builder_.get(), *scene_graph_, lcm_);
  }
}

template <typename T>
AutomotiveSimulator<T>::~AutomotiveSimulator() {}

template <typename T>
lcm::DrakeLcmInterface* AutomotiveSimulator<T>::get_lcm() {
  return owned_lcm_.get();
}

template <typename T>
systems::DiagramBuilder<T>* AutomotiveSimulator<T>::get_builder() {
  DRAKE_DEMAND(!has_started());
  return builder_.get();
}

template <typename T>
void AutomotiveSimulator<T>::ConnectCarOutputsAndPriusVis(
    int id,
    const OutputPort<T>& pose_output,
    const OutputPort<T>& velocity_output) {
  DRAKE_DEMAND(&pose_output.get_system() == &velocity_output.get_system());
  const std::string name = pose_output.get_system().get_name();
  auto ports = aggregator_->AddSinglePoseAndVelocityInput(name, id);
  builder_->Connect(pose_output, ports.pose_input_port);
  builder_->Connect(velocity_output, ports.velocity_input_port);

  if (!scene_graph_) {
    return;
  }

  // Register a (visual only) geometry frame for us to fiddle with.
  const geometry::SourceId source_id = scene_graph_->RegisterSource(
      "car_" + std::to_string(id));
  const geometry::FrameId frame_id = scene_graph_->RegisterFrame(
      source_id,
      geometry::GeometryFrame("car_origin"));

  // Defines the distance between the SDF model's origin and the middle of the
  // rear axle.  This offset is necessary because the pose_output frame's
  // origin is centered at the midpoint of the vehicle's rear axle, whereas the
  // prius.sdf model's origin is centered in the middle of a body called
  // "chassis_floor". The axes of the two frames are parallel with each
  // other. However, the distance between the origins of the two frames is
  // 1.40948 m along the model's x-axis.  We need to apply this offset to the
  // position literals that we've transcribed from the SDF.
  const double kVisOffsetInX{1.40948};

  // Add visualization geometry.
  // TODO(jeremy.nimmer) These hard-coded shapes duplicate the contents of the
  // SDF.  We should try to load the SDF instead, but there is not any geometry
  // code exposed for that yet. Related to this is the assignment of default
  // illustration properties.
  geometry::GeometryId geometry_id = scene_graph_->RegisterGeometry(
      source_id, frame_id,
      std::make_unique<geometry::GeometryInstance>(
          Isometry3d(Eigen::Translation3d(
              -2.27 + kVisOffsetInX, -0.911, -0.219 + 0.385)),
          std::make_unique<geometry::Mesh>(
              FindResourceOrThrow("drake/automotive/models/prius/prius.obj")),
          "prius_body"));
  scene_graph_->AssignRole(source_id, geometry_id,
                           geometry::IllustrationProperties());
  const geometry::IllustrationProperties grey =
      geometry::MakeDrakeVisualizerProperties(
          Eigen::Vector4d(0.2, 0.2, 0.2, 1.0));
  geometry_id = scene_graph_->RegisterGeometry(
      source_id, frame_id,
      std::make_unique<geometry::GeometryInstance>(
          math::RigidTransformd(
              math::RotationMatrixd(math::RollPitchYawd(M_PI_2, 0.0, 0.0)),
              Eigen::Vector3d(1.409 + kVisOffsetInX, 0.802, 0.316))
          .GetAsIsometry3(),
          std::make_unique<geometry::Cylinder>(0.323, 0.215),
          "left_wheel"));
  scene_graph_->AssignRole(source_id, geometry_id, grey);
  geometry_id = scene_graph_->RegisterGeometry(
      source_id, frame_id,
      std::make_unique<geometry::GeometryInstance>(
          math::RigidTransformd(
              math::RotationMatrixd(math::RollPitchYawd(M_PI_2, 0.0, 0.0)),
              Eigen::Vector3d(1.409 + kVisOffsetInX, -0.802, 0.316))
          .GetAsIsometry3(),
          std::make_unique<geometry::Cylinder>(0.323, 0.215),
          "right_wheel"));
  scene_graph_->AssignRole(source_id, geometry_id, grey);
  geometry_id = scene_graph_->RegisterGeometry(
      source_id, frame_id,
      std::make_unique<geometry::GeometryInstance>(
          math::RigidTransformd(
              math::RotationMatrixd(math::RollPitchYawd(M_PI_2, 0.0, 0.0)),
              Eigen::Vector3d(-1.409 + kVisOffsetInX, 0.802, 0.323))
          .GetAsIsometry3(),
          std::make_unique<geometry::Cylinder>(0.323, 0.215),
          "left_wheel_rear"));
  scene_graph_->AssignRole(source_id, geometry_id, grey);
  geometry_id = scene_graph_->RegisterGeometry(
      source_id, frame_id,
      std::make_unique<geometry::GeometryInstance>(
          math::RigidTransformd(
              math::RotationMatrixd(math::RollPitchYawd(M_PI_2, 0.0, 0.0)),
              Eigen::Vector3d(-1.409 + kVisOffsetInX, -0.802, 0.323))
          .GetAsIsometry3(),
          std::make_unique<geometry::Cylinder>(0.323, 0.215),
          "right_wheel_rear"));
  scene_graph_->AssignRole(source_id, geometry_id, grey);

  // Convert PoseVector into FramePoseVector.
  using Converter = systems::rendering::RenderPoseToGeometryPose<T>;
  auto* converter = builder_->template AddSystem<Converter>(
      source_id, frame_id);

  // Cascade the needful.
  builder_->Connect(pose_output, converter->get_input_port());
  builder_->Connect(converter->get_output_port(),
                    scene_graph_->get_source_pose_port(source_id));
}

// TODO(jwnimmer-tri): Modify the various vehicle model systems to be more
// uniform so common code from the following AddFooCar() methods can be moved
// into a shared method.

template <typename T>
int AutomotiveSimulator<T>::AddPriusSimpleCar(
    const std::string& name,
    const std::string& channel_name,
    const SimpleCarState<T>& initial_state) {
  DRAKE_DEMAND(!has_started());
  DRAKE_DEMAND(aggregator_ != nullptr);
  CheckNameUniqueness(name);
  const int id = allocate_vehicle_number();

  auto simple_car = builder_->template AddSystem<SimpleCar<T>>();
  simple_car->set_name(name);
  vehicles_[id] = simple_car;
  simple_car_initial_states_[simple_car].set_value(initial_state.get_value());

  ConnectCarOutputsAndPriusVis(id, simple_car->pose_output(),
      simple_car->velocity_output());

  if (!channel_name.empty() && lcm_) {
    auto command_subscriber = builder_->AddSystem(
        LcmSubscriberSystem::Make<drake::lcmt_driving_command_t>(
            channel_name, lcm_));
    auto command_decoder =
        builder_->template AddSystem<DrivingCommandDecoder>();
    builder_->Connect(*command_subscriber, *command_decoder);
    builder_->Connect(*command_decoder, *simple_car);
    AddPublisher(*simple_car, id);
  }
  return id;
}

template <typename T>
int AutomotiveSimulator<T>::AddMobilControlledSimpleCar(
    const std::string& name, bool initial_with_s, ScanStrategy path_or_branches,
    RoadPositionStrategy road_position_strategy, double period_sec,
    const SimpleCarState<T>& initial_state) {
  DRAKE_DEMAND(!has_started());
  DRAKE_DEMAND(aggregator_ != nullptr);
  CheckNameUniqueness(name);
  if (road_ == nullptr) {
    throw std::runtime_error(
        "AutomotiveSimulator::AddMobilControlledSimpleCar(): "
        "RoadGeometry not set. Please call SetRoadGeometry() first before "
        "calling this method.");
  }
  const int id = allocate_vehicle_number();

  auto mobil_planner =
      builder_->template AddSystem<MobilPlanner<T>>(*road_, initial_with_s,
                                                    road_position_strategy,
                                                    period_sec);
  mobil_planner->set_name(name + "_mobil_planner");
  auto idm_controller =
      builder_->template AddSystem<IdmController<T>>(*road_, path_or_branches,
                                                     road_position_strategy,
                                                     period_sec);
  idm_controller->set_name(name + "_idm_controller");

  auto simple_car = builder_->template AddSystem<SimpleCar<T>>();
  simple_car->set_name(name + "_simple_car");
  vehicles_[id] = simple_car;
  simple_car_initial_states_[simple_car].set_value(initial_state.get_value());
  auto pursuit = builder_->template AddSystem<PurePursuitController<T>>();
  pursuit->set_name(name + "_pure_pursuit_controller");
  auto mux = builder_->template AddSystem<DrivingCommandMux<T>>();
  mux->set_name(name + "_mux");

  // Wire up MobilPlanner and IdmController.
  builder_->Connect(simple_car->pose_output(), mobil_planner->ego_pose_input());
  builder_->Connect(simple_car->velocity_output(),
                    mobil_planner->ego_velocity_input());
  builder_->Connect(idm_controller->acceleration_output(),
                    mobil_planner->ego_acceleration_input());
  builder_->Connect(aggregator_->get_output_port(0),
                    mobil_planner->traffic_input());

  builder_->Connect(simple_car->pose_output(),
                    idm_controller->ego_pose_input());
  builder_->Connect(simple_car->velocity_output(),
                    idm_controller->ego_velocity_input());
  builder_->Connect(aggregator_->get_output_port(0),
                    idm_controller->traffic_input());

  builder_->Connect(simple_car->pose_output(), pursuit->ego_pose_input());
  builder_->Connect(mobil_planner->lane_output(), pursuit->lane_input());
  // Build DrivingCommand via a mux of two scalar outputs (a BasicVector where
  // row 0 = steering command, row 1 = acceleration command).
  builder_->Connect(pursuit->steering_command_output(), mux->steering_input());
  builder_->Connect(idm_controller->acceleration_output(),
                    mux->acceleration_input());
  builder_->Connect(mux->get_output_port(0), simple_car->get_input_port(0));

  ConnectCarOutputsAndPriusVis(id, simple_car->pose_output(),
                               simple_car->velocity_output());

  if (lcm_) {
    AddPublisher(*simple_car, id);
  }
  return id;
}

template <typename T>
int AutomotiveSimulator<T>::AddPriusTrajectoryCar(
    const std::string& name,
    const Curve2<double>& curve,
    double speed,
    double start_position) {
  DRAKE_DEMAND(!has_started());
  DRAKE_DEMAND(aggregator_ != nullptr);
  CheckNameUniqueness(name);
  const int id = allocate_vehicle_number();

  auto trajectory_car =
      builder_->template AddSystem<TrajectoryCar<T>>(curve);
  trajectory_car->set_name(name);
  vehicles_[id] = trajectory_car;

  TrajectoryCarState<double> initial_state;
  initial_state.set_position(start_position);
  initial_state.set_speed(speed);
  trajectory_car_initial_states_[trajectory_car].set_value(
      initial_state.get_value());

  ConnectCarOutputsAndPriusVis(id, trajectory_car->pose_output(),
      trajectory_car->velocity_output());

  if (lcm_) {
    AddPublisher(*trajectory_car, id);
  }
  return id;
}

template <typename T>
int AutomotiveSimulator<T>::AddIdmControlledCar(
    const std::string& name, bool initial_with_s,
    const SimpleCarState<T>& initial_state,
    const maliput::api::Lane* goal_lane, ScanStrategy path_or_branches,
    RoadPositionStrategy road_position_strategy, double period_sec) {
  DRAKE_DEMAND(!has_started());
  DRAKE_DEMAND(aggregator_ != nullptr);
  DRAKE_THROW_UNLESS(goal_lane != nullptr);
  DRAKE_THROW_UNLESS(FindLane(goal_lane->id().string()) != nullptr);
  if (road_ == nullptr) {
    throw std::runtime_error(
        "AutomotiveSimulator::AddIdmControlledCar(): "
        "RoadGeometry not set. Please call SetRoadGeometry() first before "
        "calling this method.");
  }

  CheckNameUniqueness(name);
  const int id = allocate_vehicle_number();

  auto idm_controller =
      builder_->template AddSystem<IdmController<T>>(*road_, path_or_branches,
                                                     road_position_strategy,
                                                     period_sec);
  idm_controller->set_name(name + "_idm_controller");

  const LaneDirection lane_direction(goal_lane, initial_with_s);
  auto lane_source =
      builder_->template AddSystem<systems::ConstantValueSource<T>>(
          Value<LaneDirection>(lane_direction));

  auto simple_car = builder_->template AddSystem<SimpleCar<T>>();
  simple_car->set_name(name + "_simple_car");
  vehicles_[id] = simple_car;

  simple_car_initial_states_[simple_car].set_value(initial_state.get_value());

  auto pursuit = builder_->template AddSystem<PurePursuitController<T>>();
  pursuit->set_name(name + "_pure_pursuit_controller");
  auto mux = builder_->template AddSystem<DrivingCommandMux<T>>();
  mux->set_name(name + "_mux");

  // Wire up the simple car and pose aggregator to IdmController.
  builder_->Connect(simple_car->pose_output(),
                    idm_controller->ego_pose_input());
  builder_->Connect(simple_car->velocity_output(),
                    idm_controller->ego_velocity_input());
  builder_->Connect(aggregator_->get_output_port(0),
                    idm_controller->traffic_input());

  // Wire up the lane source and simple car to PurePursuitController.
  builder_->Connect(simple_car->pose_output(), pursuit->ego_pose_input());
  builder_->Connect(lane_source->get_output_port(0), pursuit->lane_input());
  // Build DrivingCommand via a mux of two scalar outputs (a BasicVector where
  // row 0 = steering command, row 1 = acceleration command).
  builder_->Connect(pursuit->steering_command_output(),
                    mux->steering_input());
  builder_->Connect(idm_controller->acceleration_output(),
                    mux->acceleration_input());
  builder_->Connect(mux->get_output_port(0), simple_car->get_input_port(0));

  ConnectCarOutputsAndPriusVis(id, simple_car->pose_output(),
                               simple_car->velocity_output());
  if (lcm_) {
    AddPublisher(*simple_car, id);
  }

  return id;
}

template <typename T>
int AutomotiveSimulator<T>::AddPriusMaliputRailcar(
    const std::string& name,
    const LaneDirection& initial_lane_direction,
    const MaliputRailcarParams<T>& params,
    const MaliputRailcarState<T>& initial_state) {
  DRAKE_DEMAND(!has_started());
  DRAKE_DEMAND(aggregator_ != nullptr);
  CheckNameUniqueness(name);
  if (road_ == nullptr) {
    throw std::runtime_error("AutomotiveSimulator::AddPriusMaliputRailcar(): "
        "RoadGeometry not set. Please call SetRoadGeometry() first before "
        "calling this method.");
  }
  if (initial_lane_direction.lane == nullptr) {
    throw std::runtime_error("AutomotiveSimulator::AddPriusMaliputRailcar(): "
        "The provided initial lane is nullptr.");
  }
  if (initial_lane_direction.lane->segment()->junction()->road_geometry() !=
      road_.get()) {
    throw std::runtime_error("AutomotiveSimulator::AddPriusMaliputRailcar(): "
        "The provided initial lane is not within this simulation's "
        "RoadGeometry.");
  }

  const int id = allocate_vehicle_number();

  auto railcar =
      builder_->template AddSystem<MaliputRailcar<T>>(initial_lane_direction);
  railcar->set_name(name);
  vehicles_[id] = railcar;
  railcar_configs_[railcar].first.set_value(params.get_value());
  railcar_configs_[railcar].second.set_value(initial_state.get_value());

  ConnectCarOutputsAndPriusVis(id, railcar->pose_output(),
      railcar->velocity_output());
  return id;
}

template <typename T>
int AutomotiveSimulator<T>::AddIdmControlledPriusMaliputRailcar(
    const std::string& name,
    const LaneDirection& initial_lane_direction,
    ScanStrategy path_or_branches,
    RoadPositionStrategy road_position_strategy, double period_sec,
    const MaliputRailcarParams<T>& params,
    const MaliputRailcarState<T>& initial_state) {
  const int id = AddPriusMaliputRailcar(name, initial_lane_direction, params,
                                        initial_state);
  const MaliputRailcar<T>* railcar =
      dynamic_cast<const MaliputRailcar<T>*>(vehicles_.at(id));
  DRAKE_DEMAND(railcar != nullptr);
  auto controller =
      builder_->template AddSystem<IdmController<T>>(*road_, path_or_branches,
                                                     road_position_strategy,
                                                     period_sec);
  controller->set_name(name + "_IdmController");

  builder_->Connect(railcar->pose_output(), controller->ego_pose_input());
  builder_->Connect(railcar->velocity_output(),
                    controller->ego_velocity_input());
  builder_->Connect(aggregator_->get_output_port(0),
                    controller->traffic_input());
  builder_->Connect(controller->acceleration_output(),
                    railcar->command_input());
  return id;
}

template <typename T>
void AutomotiveSimulator<T>::SetMaliputRailcarAccelerationCommand(int id,
    double acceleration) {
  DRAKE_DEMAND(has_started());
  const auto iterator = vehicles_.find(id);
  if (iterator == vehicles_.end()) {
    throw std::runtime_error("AutomotiveSimulator::"
        "SetMaliputRailcarAccelerationCommand(): Failed to find vehicle with "
        "id " + std::to_string(id) + ".");
  }
  MaliputRailcar<T>* railcar = dynamic_cast<MaliputRailcar<T>*>(
      iterator->second);
  if (railcar == nullptr) {
    throw std::runtime_error("AutomotiveSimulator::"
        "SetMaliputRailcarAccelerationCommand(): The vehicle with "
        "id " + std::to_string(id) + " was not a MaliputRailcar.");
  }
  DRAKE_ASSERT(diagram_ != nullptr);
  DRAKE_ASSERT(simulator_ != nullptr);
  systems::Context<T>& context = diagram_->GetMutableSubsystemContext(
      *railcar, &simulator_->get_mutable_context());
  context.FixInputPort(railcar->command_input().get_index(),
      systems::BasicVector<double>::Make(acceleration));
}

template <typename T>
const RoadGeometry* AutomotiveSimulator<T>::SetRoadGeometry(
    std::unique_ptr<const RoadGeometry> road) {
  DRAKE_DEMAND(!has_started());
  road_ = std::move(road);
  AddRoadNetworkToSceneGraph();
  return road_.get();
}

template <typename T>
const maliput::api::Lane* AutomotiveSimulator<T>::FindLane(
    const std::string& name) const {
  if (road_ == nullptr) {
    throw std::runtime_error("AutomotiveSimulator::FindLane(): RoadGeometry "
        "not set. Please call SetRoadGeometry() first before calling this "
        "method.");
  }
  for (int i = 0; i < road_->num_junctions(); ++i) {
    const maliput::api::Junction* junction = road_->junction(i);
    for (int j = 0; j < junction->num_segments(); ++j) {
      const maliput::api::Segment* segment = junction->segment(j);
      for (int k = 0; k < segment->num_lanes(); ++k) {
        const maliput::api::Lane* lane = segment->lane(k);
        if (lane->id() == LaneId(name)) {
          return lane;
        }
      }
    }
  }
  throw std::runtime_error("AutomotiveSimulator::FindLane(): Failed to find "
      "lane named \"" + name + "\".");
}

template <typename T>
void AutomotiveSimulator<T>::AddRoadNetworkToSceneGraph() {
  if (!scene_graph_) {
    return;
  }

  // Convert the road surfaces to a visualization mesh.
  std::string filename = road_->id().string();
  std::transform(filename.begin(), filename.end(), filename.begin(),
                 [](char ch) { return ch == ' ' ? '_' : ch; });
  const std::string tmpdir = drake::temp_directory();
  maliput::utility::GenerateObjFile(road_.get(), tmpdir, filename, {});
  const std::string obj_filepath = tmpdir + "/" + filename + ".obj";

  // Add to scene graph
  scene_graph_->RegisterAnchoredGeometry(
      scene_graph_->RegisterSource("road_network"),
      std::make_unique<geometry::GeometryInstance>(
          math::RigidTransformd().GetAsIsometry3(),
          std::make_unique<geometry::Mesh>(obj_filepath),
          "road_network_obj"));
}

template <typename T>
void AutomotiveSimulator<T>::AddPublisher(const MaliputRailcar<T>& system,
                                          int vehicle_number) {
  DRAKE_DEMAND(!has_started());
  DRAKE_DEMAND(lcm_ != nullptr);
  const std::string channel =
      std::to_string(vehicle_number) + "_MALIPUT_RAILCAR_STATE";
  auto encoder = builder_->template AddSystem<MaliputRailcarStateEncoder>();
  auto publisher = builder_->AddSystem(
      LcmPublisherSystem::Make<lcmt_maliput_railcar_state_t>(
          channel, lcm_));
  builder_->Connect(system.state_output(), encoder->get_input_port(0));
  builder_->Connect(encoder->get_output_port(0), publisher->get_input_port());
}

template <typename T>
void AutomotiveSimulator<T>::AddPublisher(const SimpleCar<T>& system,
                                          int vehicle_number) {
  DRAKE_DEMAND(!has_started());
  DRAKE_DEMAND(lcm_ != nullptr);
  const std::string channel =
      std::to_string(vehicle_number) + "_SIMPLE_CAR_STATE";
  auto encoder = builder_->template AddSystem<SimpleCarStateEncoder>();
  auto publisher = builder_->AddSystem(
      LcmPublisherSystem::Make<lcmt_simple_car_state_t>(
          channel, lcm_));
  builder_->Connect(system.state_output(), encoder->get_input_port(0));
  builder_->Connect(encoder->get_output_port(0), publisher->get_input_port());
}

template <typename T>
void AutomotiveSimulator<T>::AddPublisher(const TrajectoryCar<T>& system,
                                          int vehicle_number) {
  DRAKE_DEMAND(!has_started());
  DRAKE_DEMAND(lcm_ != nullptr);
  const std::string channel =
      std::to_string(vehicle_number) + "_SIMPLE_CAR_STATE";
  auto encoder = builder_->template AddSystem<SimpleCarStateEncoder>();
  auto publisher = builder_->AddSystem(
      LcmPublisherSystem::Make<lcmt_simple_car_state_t>(
          channel, lcm_));
  builder_->Connect(system.raw_pose_output(), encoder->get_input_port(0));
  builder_->Connect(encoder->get_output_port(0), publisher->get_input_port());
}

template <typename T>
systems::System<T>& AutomotiveSimulator<T>::GetBuilderSystemByName(
    std::string name) {
  DRAKE_DEMAND(!has_started());
  systems::System<T>* result{nullptr};
  for (systems::System<T>* system : builder_->GetMutableSystems()) {
    if (system->get_name() == name) {
      DRAKE_THROW_UNLESS(!result);
      result = system;
    }
  }
  DRAKE_THROW_UNLESS(result);
  return *result;
}

template <typename T>
const systems::System<T>& AutomotiveSimulator<T>::GetDiagramSystemByName(
    std::string name) const {
  DRAKE_DEMAND(has_started());
  const systems::System<T>* result{nullptr};
  for (const systems::System<T>* system : diagram_->GetSystems()) {
    if (system->get_name() == name) {
      DRAKE_THROW_UNLESS(!result);
      result = system;
    }
  }
  DRAKE_THROW_UNLESS(result);
  return *result;
}

template <typename T>
void AutomotiveSimulator<T>::Build() {
  DRAKE_DEMAND(diagram_ == nullptr);

  pose_bundle_output_port_ =
      builder_->ExportOutput(aggregator_->get_output_port(0));

  diagram_ = builder_->Build();
  diagram_->set_name("AutomotiveSimulator");
}

template <typename T>
void AutomotiveSimulator<T>::BuildAndInitialize(
    std::unique_ptr<systems::Context<double>> initial_context) {
  DRAKE_DEMAND(!has_started());
  if (diagram_ == nullptr) {
    Build();
  }
  simulator_ = std::make_unique<systems::Simulator<T>>(*diagram_);

  if (initial_context == nullptr) {
    InitializeTrajectoryCars();
    InitializeSimpleCars();
    InitializeMaliputRailcars();
  } else {
    simulator_->reset_context(std::move(initial_context));
  }
}

template <typename T>
void AutomotiveSimulator<T>::Start(
    double target_realtime_rate,
    std::unique_ptr<systems::Context<double>> initial_context) {
  DRAKE_DEMAND(!has_started());

  BuildAndInitialize(std::move(initial_context));

  simulator_->set_target_realtime_rate(target_realtime_rate);
  const double max_step_size = 0.01;
  simulator_->template reset_integrator<RungeKutta2Integrator<T>>(
      *diagram_, max_step_size, &simulator_->get_mutable_context());
  simulator_->get_mutable_integrator().set_fixed_step_mode(true);
  simulator_->Initialize();
}

template <typename T>
void AutomotiveSimulator<T>::InitializeTrajectoryCars() {
  for (const auto& pair : trajectory_car_initial_states_) {
    const TrajectoryCar<T>* const car = pair.first;
    const TrajectoryCarState<T>& initial_state = pair.second;

    systems::VectorBase<T>& context_state =
        diagram_->GetMutableSubsystemContext(*car,
                                             &simulator_->get_mutable_context())
        .get_mutable_continuous_state_vector();
    TrajectoryCarState<T>* const state =
        dynamic_cast<TrajectoryCarState<T>*>(&context_state);
    DRAKE_ASSERT(state);
    state->set_value(initial_state.get_value());
  }
}

template <typename T>
void AutomotiveSimulator<T>::InitializeSimpleCars() {
  for (const auto& pair : simple_car_initial_states_) {
    const SimpleCar<T>* const car = pair.first;
    const SimpleCarState<T>& initial_state = pair.second;

    systems::VectorBase<T>& context_state =
        diagram_->GetMutableSubsystemContext(*car,
                                             &simulator_->get_mutable_context())
        .get_mutable_continuous_state_vector();
    SimpleCarState<T>* const state =
        dynamic_cast<SimpleCarState<T>*>(&context_state);
    DRAKE_ASSERT(state);
    state->set_value(initial_state.get_value());
  }
}

template <typename T>
void AutomotiveSimulator<T>::InitializeMaliputRailcars() {
  for (auto& pair : railcar_configs_) {
    const MaliputRailcar<T>* const car = pair.first;
    const MaliputRailcarParams<T>& params = pair.second.first;
    const MaliputRailcarState<T>& initial_state = pair.second.second;

    systems::Context<T>& context = diagram_->GetMutableSubsystemContext(
         *car, &simulator_->get_mutable_context());

    systems::VectorBase<T>& context_state =
        context.get_mutable_continuous_state_vector();
    MaliputRailcarState<T>* const state =
        dynamic_cast<MaliputRailcarState<T>*>(&context_state);
    DRAKE_ASSERT(state);
    state->set_value(initial_state.get_value());

    MaliputRailcarParams<T>& railcar_system_params =
        car->get_mutable_parameters(&context);
    railcar_system_params.set_value(params.get_value());
  }
}

template <typename T>
void AutomotiveSimulator<T>::StepBy(const T& time_step) {
  const T time = simulator_->get_context().get_time();
  SPDLOG_TRACE(drake::log(), "Time is now {}", time);
  simulator_->AdvanceTo(time + time_step);
}

template <typename T>
int AutomotiveSimulator<T>::allocate_vehicle_number() {
  DRAKE_DEMAND(!has_started());
  return next_vehicle_number_++;
}

template <typename T>
void AutomotiveSimulator<T>::CheckNameUniqueness(const std::string& name) {
  for (const auto& vehicle : vehicles_) {
    if (vehicle.second->get_name() == name) {
      throw std::runtime_error("A vehicle named \"" + name + "\" already "
          "exists. It has id " + std::to_string(vehicle.first) + ".");
    }
  }
}

template <typename T>
PoseBundle<T> AutomotiveSimulator<T>::GetCurrentPoses() const {
  DRAKE_DEMAND(has_started());
  const auto& context = simulator_->get_context();
  const auto& pose_port = diagram_->get_output_port(pose_bundle_output_port_);
  return pose_port.template Eval<PoseBundle<T>>(context);
}

template class AutomotiveSimulator<double>;

}  // namespace automotive
}  // namespace drake
