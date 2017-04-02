#include "drake/automotive/automotive_simulator.h"

#include <utility>

#include "drake/automotive/dev/endless_road_oracle.h"
#include "drake/automotive/gen/driving_command_translator.h"
#include "drake/automotive/gen/endless_road_car_state_translator.h"
#include "drake/automotive/gen/euler_floating_joint_state_translator.h"
#include "drake/automotive/gen/simple_car_state_translator.h"
#include "drake/automotive/maliput/utility/generate_urdf.h"
#include "drake/automotive/prius_vis.h"
#include "drake/common/drake_throw.h"
#include "drake/common/text_logging.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_viewer_draw.hpp"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/create_load_robot_message.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"

namespace drake {

using multibody::joints::kRollPitchYaw;
using systems::lcm::LcmPublisherSystem;

namespace automotive {

template <typename T>
AutomotiveSimulator<T>::AutomotiveSimulator()
    : AutomotiveSimulator(std::make_unique<lcm::DrakeLcm>()) {}

template <typename T>
AutomotiveSimulator<T>::AutomotiveSimulator(
    std::unique_ptr<lcm::DrakeLcmInterface> lcm)
    : lcm_(std::move(lcm)) {}

template <typename T>
AutomotiveSimulator<T>::~AutomotiveSimulator() {
  // Forces the LCM instance to be destroyed before any of the subscribers are
  // destroyed.
  lcm_.reset();
}

template <typename T>
lcm::DrakeLcmInterface* AutomotiveSimulator<T>::get_lcm() {
  return lcm_.get();
}

template <typename T>
systems::DiagramBuilder<T>* AutomotiveSimulator<T>::get_builder() {
  DRAKE_DEMAND(!has_started());
  return builder_.get();
}

template <typename T>
int AutomotiveSimulator<T>::AddPriusSimpleCar(
    const std::string& model_name,
    const std::string& channel_name,
    const SimpleCarState<T>& initial_state) {
  DRAKE_DEMAND(!has_started());
  DRAKE_DEMAND(aggregator_ != nullptr);
  const int id = allocate_vehicle_number();

  static const DrivingCommandTranslator driving_command_translator;
  DRAKE_DEMAND(!channel_name.empty());
  auto command_subscriber =
      builder_->template AddSystem<systems::lcm::LcmSubscriberSystem>(
          channel_name, driving_command_translator, lcm_.get());
  auto simple_car = builder_->template AddSystem<SimpleCar<T>>();
  simple_car_initial_states_[simple_car].set_value(initial_state.get_value());
  auto coord_transform =
      builder_->template AddSystem<SimpleCarToEulerFloatingJoint<T>>();
  const auto& descriptor = aggregator_->AddSingleInput(model_name, id);
  builder_->Connect(simple_car->pose_output(),
                    aggregator_->get_input_port(descriptor.get_index()));

  builder_->Connect(*command_subscriber, *simple_car);
  builder_->Connect(simple_car->state_output(),
                    coord_transform->get_input_port(0));
  AddPublisher(*simple_car, id);
  AddPublisher(*coord_transform, id);
  car_vis_applicator_->AddCarVis(std::make_unique<PriusVis<T>>(id, model_name));
  return id;
}

template <typename T>
int AutomotiveSimulator<T>::AddPriusTrajectoryCar(
    const Curve2<double>& curve,
    double speed,
    double start_time) {
  DRAKE_DEMAND(!has_started());
  DRAKE_DEMAND(aggregator_ != nullptr);
  const int id = allocate_vehicle_number();
  const std::string model_name =
      "trajectory_car_" + std::to_string(id);

  auto trajectory_car =
      builder_->template AddSystem<TrajectoryCar<T>>(curve, speed, start_time);
  auto coord_transform =
      builder_->template AddSystem<SimpleCarToEulerFloatingJoint<T>>();
  const auto& descriptor = aggregator_->AddSingleInput(model_name, id);
  builder_->Connect(trajectory_car->pose_output(),
                    aggregator_->get_input_port(descriptor.get_index()));

  builder_->Connect(trajectory_car->raw_pose_output(),
                    coord_transform->get_input_port(0));
  AddPublisher(*trajectory_car, id);
  AddPublisher(*coord_transform, id);
  car_vis_applicator_->AddCarVis(
      std::make_unique<PriusVis<T>>(id, model_name));
  return id;
}

template <typename T>
int AutomotiveSimulator<T>::AddPriusEndlessRoadCar(
    const std::string& name,
    const EndlessRoadCarState<T>& initial_state,
    typename EndlessRoadCar<T>::ControlType control_type,
    const std::string& channel_name) {
  DRAKE_DEMAND(!has_started());
  DRAKE_DEMAND(endless_road_ != nullptr);
  const int id = allocate_vehicle_number();
  const std::string model_name = "endless_road_car_" + std::to_string(id);

  auto endless_road_car = builder_->template AddSystem<EndlessRoadCar<T>>(
      name, endless_road_.get(), control_type);
  auto coord_transform =
      builder_->template AddSystem<EndlessRoadCarToEulerFloatingJoint<T>>(
          endless_road_.get());

  // Save the desired initial state in order to initialize the Simulator later,
  // when we have a Simulator to initialize.
  endless_road_cars_[endless_road_car].set_value(initial_state.get_value());

  switch (control_type) {
    case EndlessRoadCar<T>::kNone: {
      break;
    }
    case EndlessRoadCar<T>::kUser: {
      DRAKE_DEMAND(!channel_name.empty());
      static const DrivingCommandTranslator driving_command_translator;
      auto command_subscriber =
          builder_->template AddSystem<systems::lcm::LcmSubscriberSystem>(
              channel_name, driving_command_translator, lcm_.get());
      builder_->Connect(*command_subscriber, *endless_road_car);
      break;
    }
    case EndlessRoadCar<T>::kIdm: {
      // Nothing to do here.  At Start(), we will construct the central
      // EndlessRoadOracle and attach it to endless_road_cars_ as needed.
      break;
    }
    default: { DRAKE_ABORT(); }
  }

  builder_->Connect(*endless_road_car, *coord_transform);
  AddPublisher(*endless_road_car, id);
  AddPublisher(*coord_transform, id);
  car_vis_applicator_->AddCarVis(
    std::make_unique<PriusVis<T>>(id, model_name));
  return id;
}

template <typename T>
const maliput::api::RoadGeometry* AutomotiveSimulator<T>::SetRoadGeometry(
    std::unique_ptr<const maliput::api::RoadGeometry> road) {
  DRAKE_DEMAND(!has_started());
  road_ = std::move(road);
  GenerateAndLoadRoadNetworkUrdf();
  return road_.get();
}

template <typename T>
const maliput::utility::InfiniteCircuitRoad*
AutomotiveSimulator<T>::SetRoadGeometry(
    std::unique_ptr<const maliput::api::RoadGeometry> road,
    const maliput::api::LaneEnd& start,
    const std::vector<const maliput::api::Lane*>& path) {
  DRAKE_DEMAND(!has_started());
  road_ = std::move(road);
  endless_road_ = std::make_unique<maliput::utility::InfiniteCircuitRoad>(
      maliput::api::RoadGeometryId({"ForeverRoad"}),
      road_.get(), start, path);
  GenerateAndLoadRoadNetworkUrdf();
  return endless_road_.get();
}

template <typename T>
void AutomotiveSimulator<T>::GenerateAndLoadRoadNetworkUrdf() {
  maliput::utility::GenerateUrdfFile(road_.get(),
                                     "/tmp", road_->id().id,
                                     maliput::utility::ObjFeatures());
  const std::string urdf_filepath =
      std::string("/tmp/") + road_->id().id + ".urdf";
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      urdf_filepath,
      drake::multibody::joints::kFixed,
      tree_.get());
}

template <typename T>
void AutomotiveSimulator<T>::AddPublisher(const SimpleCar<T>& system,
                                          int vehicle_number) {
  DRAKE_DEMAND(!has_started());
  static const SimpleCarStateTranslator translator;
  auto publisher =
      builder_->template AddSystem<LcmPublisherSystem>(
          std::to_string(vehicle_number) + "_SIMPLE_CAR_STATE", translator,
          lcm_.get());
  builder_->Connect(system.state_output(), publisher->get_input_port(0));
}

template <typename T>
void AutomotiveSimulator<T>::AddPublisher(const TrajectoryCar<T>& system,
                                          int vehicle_number) {
  DRAKE_DEMAND(!has_started());
  static const SimpleCarStateTranslator translator;
  auto publisher =
      builder_->template AddSystem<LcmPublisherSystem>(
          std::to_string(vehicle_number) + "_SIMPLE_CAR_STATE", translator,
          lcm_.get());
  builder_->Connect(system.raw_pose_output(), publisher->get_input_port(0));
}

template <typename T>
void AutomotiveSimulator<T>::AddPublisher(const EndlessRoadCar<T>& system,
                                          int vehicle_number) {
  DRAKE_DEMAND(!has_started());
  static const EndlessRoadCarStateTranslator translator;
  auto publisher =
      builder_->template AddSystem<LcmPublisherSystem>(
          std::to_string(vehicle_number) + "_ENDLESS_ROAD_CAR_STATE",
          translator, lcm_.get());
  builder_->Connect(system, *publisher);
}

template <typename T>
void AutomotiveSimulator<T>::AddPublisher(
    const EndlessRoadCarToEulerFloatingJoint<T>& system,
    int vehicle_number) {
  DRAKE_DEMAND(!has_started());
  static const EulerFloatingJointStateTranslator translator;
  auto publisher =
      builder_->template AddSystem<LcmPublisherSystem>(
          std::to_string(vehicle_number) + "_FLOATING_JOINT_STATE",
          translator, lcm_.get());
  builder_->Connect(system, *publisher);
}

template <typename T>
void AutomotiveSimulator<T>::AddPublisher(
    const SimpleCarToEulerFloatingJoint<T>& system, int vehicle_number) {
  DRAKE_DEMAND(!has_started());
  static const EulerFloatingJointStateTranslator translator;
  auto publisher =
      builder_->template AddSystem<LcmPublisherSystem>(
          std::to_string(vehicle_number) + "_FLOATING_JOINT_STATE", translator,
          lcm_.get());
  builder_->Connect(system, *publisher);
}

template <typename T>
void AutomotiveSimulator<T>::AddSystem(
    std::unique_ptr<systems::System<T>> system) {
  DRAKE_DEMAND(!has_started());
  builder_->AddSystem(std::move(system));
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
void AutomotiveSimulator<T>::TransmitLoadMessage() {
  const lcmt_viewer_load_robot load_car_message =
      car_vis_applicator_->get_load_robot_message();
  const lcmt_viewer_load_robot load_terrain_message =
      multibody::CreateLoadRobotMessage<T>(*tree_);
  lcmt_viewer_load_robot load_message;
  load_message.num_links = load_car_message.num_links +
                           load_terrain_message.num_links;
  for (int i = 0; i < load_car_message.num_links; ++i) {
    load_message.link.push_back(load_car_message.link.at(i));
  }
  for (int i = 0; i < load_terrain_message.num_links; ++i) {
    load_message.link.push_back(load_terrain_message.link.at(i));
  }
  SendLoadRobotMessage(load_message);
}

template <typename T>
void AutomotiveSimulator<T>::SendLoadRobotMessage(
    const lcmt_viewer_load_robot& message) {
  const int num_bytes = message.getEncodedSize();
  std::vector<uint8_t> message_bytes(num_bytes);
  const int num_bytes_encoded =
      message.encode(message_bytes.data(), 0, num_bytes);
  DRAKE_ASSERT(num_bytes_encoded == num_bytes);
  lcm_->Publish("DRAKE_VIEWER_LOAD_ROBOT", message_bytes.data(), num_bytes);
}

template <typename T>
void AutomotiveSimulator<T>::Start(double target_realtime_rate) {
  DRAKE_DEMAND(!has_started());
  TransmitLoadMessage();

  builder_->Connect(
      aggregator_->get_output_port(0),
      car_vis_applicator_->get_car_poses_input_port());
  builder_->Connect(
      car_vis_applicator_->get_visual_geometry_poses_output_port(),
      bundle_to_draw_->get_input_port(0));
  lcm_publisher_ = builder_->AddSystem(
      LcmPublisherSystem::Make<lcmt_viewer_draw>("DRAKE_VIEWER_DRAW",
                                                 lcm_.get()));
  builder_->Connect(
      bundle_to_draw_->get_output_port(0),
      lcm_publisher_->get_input_port(0));

  if (endless_road_) {
    // Now that we have all the cars, construct an appropriately tentacled
    // EndlessRoadOracle and hook everything up.
    const int num_cars = endless_road_cars_.size();

    auto oracle = builder_->template AddSystem<EndlessRoadOracle<T>>(
        endless_road_.get(), num_cars);
    int i = 0;
    for (const auto& item : endless_road_cars_) {
      const EndlessRoadCar<T>* car = item.first;

      // Every car is visible to the Oracle...
      builder_->Connect(car->get_output_port(0), oracle->get_input_port(i));
      // ...however, only IDM-controlled cars care about Oracle output.
      // TODO(maddog@tri.global)  Future Optimization:  Oracle should not
      //                          bother to compute output for cars which
      //                          will ignore it.
      switch (car->control_type()) {
        case EndlessRoadCar<T>::kNone: {
          break;  // No input.
        }
        case EndlessRoadCar<T>::kUser: {
          break;  // Already connected to controls.
        }
        case EndlessRoadCar<T>::kIdm: {
          builder_->Connect(oracle->get_output_port(i), car->get_input_port(0));
          break;
        }
        default: { DRAKE_ABORT(); }
      }
      ++i;
    }
  }

  diagram_ = builder_->Build();
  simulator_ = std::make_unique<systems::Simulator<T>>(*diagram_);

  // Initialize the state of the SimpleCars.
  for (const auto& pair : simple_car_initial_states_) {
    const SimpleCar<T>* const car = pair.first;
    const SimpleCarState<T>& initial_state = pair.second;

    systems::VectorBase<T>* context_state =
        diagram_->GetMutableSubsystemContext(simulator_->get_mutable_context(),
                                             car)
        ->get_mutable_continuous_state()->get_mutable_vector();
    SimpleCarState<T>* const state =
        dynamic_cast<SimpleCarState<T>*>(context_state);
    DRAKE_ASSERT(state);
    state->set_value(initial_state.get_value());
  }

  // Initialize the state of the EndlessRoadCars.
  for (const auto& pair : endless_road_cars_) {
    const EndlessRoadCar<T>* const car = pair.first;
    const EndlessRoadCarState<T>& initial_state = pair.second;

    systems::VectorBase<T>* context_state =
        diagram_->GetMutableSubsystemContext(simulator_->get_mutable_context(),
                                             car)
        ->get_mutable_continuous_state()->get_mutable_vector();
    EndlessRoadCarState<T>* const state =
        dynamic_cast<EndlessRoadCarState<T>*>(context_state);
    DRAKE_ASSERT(state);
    state->set_value(initial_state.get_value());
  }

  lcm_->StartReceiveThread();

  simulator_->set_target_realtime_rate(target_realtime_rate);
  simulator_->get_mutable_integrator()->set_maximum_step_size(0.01);
  simulator_->get_mutable_integrator()->set_minimum_step_size(0.01);
  simulator_->Initialize();
}

template <typename T>
void AutomotiveSimulator<T>::StepBy(const T& time_step) {
  const T time = simulator_->get_context().get_time();
  SPDLOG_TRACE(drake::log(), "Time is now {}", time);
  simulator_->StepTo(time + time_step);
}

template <typename T>
int AutomotiveSimulator<T>::allocate_vehicle_number() {
  DRAKE_DEMAND(!has_started());
  return next_vehicle_number_++;
}

template class AutomotiveSimulator<double>;

}  // namespace automotive
}  // namespace drake
