#include "drake/automotive/automotive_simulator.h"

#include <limits>

#include <lcm/lcm-cpp.hpp>

#include "drake/automotive/dev/endless_road_car.h"
#include "drake/automotive/dev/endless_road_car_to_euler_floating_joint.h"
#include "drake/automotive/dev/endless_road_oracle.h"
#include "drake/automotive/dev/infinite_circuit_road.h"
#include "drake/automotive/gen/driving_command_translator.h"
#include "drake/automotive/gen/endless_road_car_state_translator.h"
#include "drake/automotive/gen/euler_floating_joint_state_translator.h"
#include "drake/automotive/gen/simple_car_state_translator.h"
#include "drake/automotive/maliput/utility/generate_urdf.h"
#include "drake/automotive/simple_car.h"
#include "drake/automotive/simple_car_to_euler_floating_joint.h"
#include "drake/automotive/trajectory_car.h"
#include "drake/common/drake_throw.h"
#include "drake/common/text_logging.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/model_instance_id_table.h"
#include "drake/multibody/parsers/sdf_parser.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/multiplexer.h"

namespace drake {

using multibody::joints::kFixed;
using multibody::joints::kRollPitchYaw;
using systems::rendering::PoseAggregator;

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
  DRAKE_DEMAND(!started_);
  return builder_.get();
}

template <typename T>
const RigidBodyTree<T>& AutomotiveSimulator<T>::get_rigid_body_tree() {
  return *rigid_body_tree_;
}

template <typename T>
int AutomotiveSimulator<T>::AddSimpleCarFromSdf(
    const std::string& sdf_filename,
    const std::string& model_name,
    const std::string& channel_name,
    const SimpleCarState<T>& initial_state) {
  DRAKE_DEMAND(!started_);
  DRAKE_DEMAND(aggregator_ != nullptr);
  const int vehicle_number = allocate_vehicle_number();

  static const DrivingCommandTranslator driving_command_translator;
  DRAKE_DEMAND(!channel_name.empty());
  auto command_subscriber =
      builder_->template AddSystem<systems::lcm::LcmSubscriberSystem>(
          channel_name, driving_command_translator, lcm_.get());
  auto simple_car = builder_->template AddSystem<SimpleCar<T>>();
  simple_car_initial_states_[simple_car].set_value(initial_state.get_value());
  auto coord_transform =
      builder_->template AddSystem<SimpleCarToEulerFloatingJoint<T>>();
  const auto& descriptor = aggregator_->AddSingleInput(
      "simple_car", vehicle_number);
  builder_->Connect(simple_car->pose_output(),
                    aggregator_->get_input_port(descriptor.get_index()));

  builder_->Connect(*command_subscriber, *simple_car);
  builder_->Connect(simple_car->state_output(),
                    coord_transform->get_input_port(0));
  AddPublisher(*simple_car, vehicle_number);
  AddPublisher(*coord_transform, vehicle_number);
  return AddSdfModel(sdf_filename, coord_transform, model_name);
}

template <typename T>
int AutomotiveSimulator<T>::AddTrajectoryCarFromSdf(
    const std::string& sdf_filename,
    const Curve2<double>& curve,
    double speed,
    double start_time) {
  DRAKE_DEMAND(!started_);
  DRAKE_DEMAND(aggregator_ != nullptr);
  const int vehicle_number = allocate_vehicle_number();

  auto trajectory_car =
      builder_->template AddSystem<TrajectoryCar<T>>(curve, speed, start_time);
  auto coord_transform =
      builder_->template AddSystem<SimpleCarToEulerFloatingJoint<T>>();
  const auto& descriptor = aggregator_->AddSingleInput(
      "trajectory_car", vehicle_number);
  builder_->Connect(trajectory_car->pose_output(),
                    aggregator_->get_input_port(descriptor.get_index()));

  builder_->Connect(trajectory_car->raw_pose_output(),
                    coord_transform->get_input_port(0));
  AddPublisher(*trajectory_car, vehicle_number);
  AddPublisher(*coord_transform, vehicle_number);
  return AddSdfModel(sdf_filename, coord_transform, "" /* model_name */);
}

template <typename T>
int AutomotiveSimulator<T>::AddEndlessRoadCar(
    const std::string& id,
    const std::string& sdf_filename,
    const EndlessRoadCarState<T>& initial_state,
    typename EndlessRoadCar<T>::ControlType control_type,
    const std::string& channel_name) {
  DRAKE_DEMAND(!started_);
  DRAKE_DEMAND(endless_road_ != nullptr);
  const int vehicle_number = allocate_vehicle_number();

  auto endless_road_car = builder_->template AddSystem<EndlessRoadCar<T>>(
      id, endless_road_.get(), control_type);
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
  AddPublisher(*endless_road_car, vehicle_number);
  AddPublisher(*coord_transform, vehicle_number);
  return AddSdfModel(sdf_filename, coord_transform, id);
}

template <typename T>
const maliput::api::RoadGeometry* AutomotiveSimulator<T>::SetRoadGeometry(
    std::unique_ptr<const maliput::api::RoadGeometry> road) {
  DRAKE_DEMAND(!started_);
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
  DRAKE_DEMAND(!started_);
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
      rigid_body_tree_.get());
}


namespace {
template <typename T>
int LoadSdfModel(
    const std::string& sdf_filename,
    const systems::LeafSystem<T>* coord_transform,
    const std::string& model_name,
    RigidBodyTree<T>* rigid_body_tree,
    std::vector<std::pair<int, const systems::System<T>*>
        >* rigid_body_tree_publisher_inputs) {
  const parsers::ModelInstanceIdTable table =
      parsers::sdf::AddModelInstancesFromSdfFileToWorld(
          sdf_filename, kRollPitchYaw, rigid_body_tree);
  // TODO(liang.fok): Add support for SDF files containing more than one model.
  DRAKE_DEMAND(table.size() == 1);
  const int model_instance_id = table.begin()->second;
  rigid_body_tree_publisher_inputs->push_back(
      std::make_pair(model_instance_id, coord_transform));

  if (!model_name.empty()) {
    // TODO(russt): Set the model_name instead, once
    // https://github.com/RobotLocomotion/drake/issues/3053 is resolved.
    // For now, only the body names show up in the drake visualizer.
    const std::vector<int>& body_indices =
        rigid_body_tree->FindBaseBodies(model_instance_id);
    DRAKE_DEMAND(body_indices.size() == 1);
    rigid_body_tree->bodies[body_indices.at(0)]->set_name(model_name);
  }

  return model_instance_id;
}
}  // namespace


template <typename T>
int AutomotiveSimulator<T>::AddSdfModel(
    const std::string& sdf_filename,
    const SimpleCarToEulerFloatingJoint<T>* coord_transform,
    const std::string& model_name) {
  return LoadSdfModel(sdf_filename, coord_transform, model_name,
                      rigid_body_tree_.get(),
                      &rigid_body_tree_publisher_inputs_);
}

template <typename T>
int AutomotiveSimulator<T>::AddSdfModel(
    const std::string& sdf_filename,
    const EndlessRoadCarToEulerFloatingJoint<T>* coord_transform,
    const std::string& model_name) {
  return LoadSdfModel(sdf_filename, coord_transform, model_name,
                      rigid_body_tree_.get(),
                      &rigid_body_tree_publisher_inputs_);
}

template <typename T>
void AutomotiveSimulator<T>::AddPublisher(const SimpleCar<T>& system,
                                          int vehicle_number) {
  DRAKE_DEMAND(!started_);
  static const SimpleCarStateTranslator translator;
  auto publisher =
      builder_->template AddSystem<systems::lcm::LcmPublisherSystem>(
          std::to_string(vehicle_number) + "_SIMPLE_CAR_STATE", translator,
          lcm_.get());
  builder_->Connect(system.state_output(), publisher->get_input_port(0));
}

template <typename T>
void AutomotiveSimulator<T>::AddPublisher(const TrajectoryCar<T>& system,
                                          int vehicle_number) {
  DRAKE_DEMAND(!started_);
  static const SimpleCarStateTranslator translator;
  auto publisher =
      builder_->template AddSystem<systems::lcm::LcmPublisherSystem>(
          std::to_string(vehicle_number) + "_SIMPLE_CAR_STATE", translator,
          lcm_.get());
  builder_->Connect(system.raw_pose_output(), publisher->get_input_port(0));
}

template <typename T>
void AutomotiveSimulator<T>::AddPublisher(const EndlessRoadCar<T>& system,
                                          int vehicle_number) {
  DRAKE_DEMAND(!started_);
  static const EndlessRoadCarStateTranslator translator;
  auto publisher =
      builder_->template AddSystem<systems::lcm::LcmPublisherSystem>(
          std::to_string(vehicle_number) + "_ENDLESS_ROAD_CAR_STATE",
          translator, lcm_.get());
  builder_->Connect(system, *publisher);
}

template <typename T>
void AutomotiveSimulator<T>::AddPublisher(
    const EndlessRoadCarToEulerFloatingJoint<T>& system,
    int vehicle_number) {
  DRAKE_DEMAND(!started_);
  static const EulerFloatingJointStateTranslator translator;
  auto publisher =
      builder_->template AddSystem<systems::lcm::LcmPublisherSystem>(
          std::to_string(vehicle_number) + "_FLOATING_JOINT_STATE",
          translator, lcm_.get());
  builder_->Connect(system, *publisher);
}

template <typename T>
void AutomotiveSimulator<T>::AddPublisher(
    const SimpleCarToEulerFloatingJoint<T>& system, int vehicle_number) {
  DRAKE_DEMAND(!started_);
  static const EulerFloatingJointStateTranslator translator;
  auto publisher =
      builder_->template AddSystem<systems::lcm::LcmPublisherSystem>(
          std::to_string(vehicle_number) + "_FLOATING_JOINT_STATE", translator,
          lcm_.get());
  builder_->Connect(system, *publisher);
}

template <typename T>
void AutomotiveSimulator<T>::AddSystem(
    std::unique_ptr<systems::System<T>> system) {
  DRAKE_DEMAND(!started_);
  builder_->AddSystem(std::move(system));
}

template <typename T>
systems::System<T>& AutomotiveSimulator<T>::GetBuilderSystemByName(
    std::string name) {
  DRAKE_DEMAND(!started_);
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
  DRAKE_DEMAND(started_);
  // Ask the diagram.
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
std::vector<int> AutomotiveSimulator<T>::GetModelJointStateSizes() const {
  std::vector<int> position_states;
  std::vector<int> velocity_states;

  for (const auto& model_info : rigid_body_tree_publisher_inputs_) {
    const int model_instance_id = model_info.first;
    int num_joint_positions{0};
    int num_joint_velocities{0};
    const std::vector<const RigidBody<T>*> bodies =
        rigid_body_tree_->FindModelInstanceBodies(model_instance_id);
    for (const auto& body : bodies) {
      num_joint_positions += body->getJoint().get_num_positions();
      num_joint_velocities += body->getJoint().get_num_velocities();
    }
    position_states.push_back(num_joint_positions);
    velocity_states.push_back(num_joint_velocities);
  }
  std::vector<int> result;
  result.reserve(position_states.size() + velocity_states.size());
  result.insert(result.end(), position_states.begin(), position_states.end());
  result.insert(result.end(), velocity_states.begin(), velocity_states.end());
  return result;
}

template <typename T>
void AutomotiveSimulator<T>::ConnectJointStateSourcesToVisualizer() {
  DRAKE_DEMAND(!started_);
  if (!rigid_body_tree_publisher_inputs_.empty()) {
    // Arithmetic for RigidBodyTreeLcmPublisher input sizing.  We have systems
    // that output an Euler floating joint state.  We want to mux them together
    // to feed RigidBodyTreeLcmPublisher.  We stack them up in joint order as
    // the position input to the publisher, and then also need to feed zeros
    // for all of the joint velocities.
    std::vector<int> joint_state_sizes = GetModelJointStateSizes();
    auto multiplexer = builder_->template AddSystem<systems::Multiplexer<T>>(
        joint_state_sizes);

    auto rigid_body_tree_publisher =
        builder_->template AddSystem<systems::DrakeVisualizer>(
            *rigid_body_tree_, lcm_.get());
    builder_->Connect(*multiplexer, *rigid_body_tree_publisher);

    // Connects systems that provide joint positions to the mux position inputs.
    // Connects a zero-velocity source to each of the mux velocity inputs.
    const int num_models = rigid_body_tree_publisher_inputs_.size();
    for (int model_index = 0; model_index < num_models; ++model_index) {
      int model_instance_id{};
      const systems::System<T>* model_pose_system{};
      std::tie(model_instance_id, model_pose_system) =
          rigid_body_tree_publisher_inputs_[model_index];

      // Verifies that the current model instance is connected to the world via
      // an RPY floating joint.
      //
      // TODO(liang.fok) Support models that are connected to the world via
      // non RPY floating joints. See #3919.
      const int kRpyJointNumPos{6};
      const int kRpyJointNumVel{6};

      std::vector<int> base_body_indices =
          rigid_body_tree_->FindBaseBodies(model_instance_id);
      DRAKE_DEMAND(base_body_indices.size() == 1);
      const DrakeJoint& base_joint =
          rigid_body_tree_->bodies.at(base_body_indices.at(0))->getJoint();
      DRAKE_DEMAND(base_joint.is_floating());
      DRAKE_DEMAND(base_joint.get_num_positions() == kRpyJointNumPos);
      DRAKE_DEMAND(base_joint.get_num_velocities() == kRpyJointNumVel);

      // Determines the number of DOFs in the model instance.
      int num_position_dofs = joint_state_sizes.at(model_index);
      int num_velocity_dofs = joint_state_sizes.at(num_models + model_index);

      if (num_position_dofs == kRpyJointNumPos) {
        // The robot has no position DOFs beyond the floating joint DOFs. Thus,
        // model_pose_system can be connected directly to the multiplexer.
        builder_->Connect(model_pose_system->get_output_port(0),
                          multiplexer->get_input_port(model_index));
      } else {
        // The robot has additional DOFs beyond the floating joint DOFs. Thus,
        // we need to connect a system that provides additional joint position
        // state to the multiplexer. For now, we'll connect a zero-source
        // system.
        //
        // TODO(liang.fok): Enable non-floating joint DOFs to have non-zero
        // state see: #3919.
        const int num_reg_pos_dofs = num_position_dofs - kRpyJointNumPos;
        auto zero_position_source =
            builder_->template AddSystem<systems::ConstantVectorSource>(
                VectorX<T>::Zero(num_reg_pos_dofs).eval());
        auto position_mux =
            builder_->template AddSystem<systems::Multiplexer<T>>(
                std::vector<int>{kRpyJointNumPos, num_reg_pos_dofs});
        builder_->Connect(model_pose_system->get_output_port(0),
                          position_mux->get_input_port(0));
        builder_->Connect(zero_position_source->get_output_port(),
                          position_mux->get_input_port(1));
        builder_->Connect(position_mux->get_output_port(0),
                          multiplexer->get_input_port(model_index));
      }

      // Connects a zero-vector source for the velocity state.
      auto zero_velocity_source =
          builder_->template AddSystem<systems::ConstantVectorSource>(
              VectorX<T>::Zero(num_velocity_dofs).eval());

      builder_->Connect(zero_velocity_source->get_output_port(),
                        multiplexer->get_input_port(num_models + model_index));
    }
  }
}

template <typename T>
void AutomotiveSimulator<T>::Start(double target_realtime_rate) {
  DRAKE_DEMAND(!started_);

  // By this time, all model instances should have been added to the tree.
  // While the parsers have already called `compile()` on the `RigidBodyTree`,
  // in an abundance of caution, the following line calls `compile()` again.
  rigid_body_tree_->compile();

  ConnectJointStateSourcesToVisualizer();

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

  started_ = true;
}

template <typename T>
void AutomotiveSimulator<T>::StepBy(const T& time_step) {
  const T time = simulator_->get_context().get_time();
  SPDLOG_TRACE(drake::log(), "Time is now {}", time);
  simulator_->StepTo(time + time_step);
}

template <typename T>
int AutomotiveSimulator<T>::allocate_vehicle_number() {
  DRAKE_DEMAND(!started_);
  return next_vehicle_number_++;
}

template class AutomotiveSimulator<double>;

}  // namespace automotive
}  // namespace drake
