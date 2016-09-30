#include "drake/automotive/automotive_simulator.h"

#include <lcm/lcm-cpp.hpp>

#include "drake/automotive/gen/driving_command_translator.h"
#include "drake/automotive/gen/euler_floating_joint_state_translator.h"
#include "drake/automotive/gen/simple_car_state_translator.h"
#include "drake/automotive/simple_car.h"
#include "drake/automotive/simple_car_to_euler_floating_joint.h"
#include "drake/automotive/trajectory_car.h"
#include "drake/common/drake_path.h"
#include "drake/common/drake_throw.h"
#include "drake/common/text_logging.h"
#include "drake/drakeAutomotive_export.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/primitives/constant_vector_source.h"
#include "drake/systems/framework/primitives/multiplexer.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_receive_thread.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/plants/parser_model_instance_id_table.h"
#include "drake/systems/plants/parser_urdf.h"
#include "drake/systems/plants/rigid_body_plant/rigid_body_tree_lcm_publisher.h"

namespace drake {
namespace automotive {

template <typename T>
AutomotiveSimulator<T>::AutomotiveSimulator() {}

template <typename T>
AutomotiveSimulator<T>::~AutomotiveSimulator() {}

template <typename T>
lcm::LCM* AutomotiveSimulator<T>::get_lcm() {
  return lcm_.get();
}

template <typename T>
systems::DiagramBuilder<T>* AutomotiveSimulator<T>::get_builder() {
  DRAKE_DEMAND(!started_);
  return builder_.get();
}

template <typename T>
const RigidBodyTree& AutomotiveSimulator<T>::get_rigid_body_tree() {
  return *rigid_body_tree_;
}

template <typename T>
void AutomotiveSimulator<T>::AddSimpleCar() {
  DRAKE_DEMAND(!started_);
  const int vehicle_number = allocate_vehicle_number();

  static const DrivingCommandTranslator driving_command_translator;
  auto command_subscriber =
      builder_->template AddSystem<systems::lcm::LcmSubscriberSystem>(
          "DRIVING_COMMAND", driving_command_translator, lcm_.get());
  auto simple_car = builder_->template AddSystem<SimpleCar<T>>();
  auto coord_transform =
      builder_->template AddSystem<SimpleCarToEulerFloatingJoint<T>>();

  builder_->Connect(*command_subscriber, *simple_car);
  builder_->Connect(*simple_car, *coord_transform);
  AddPublisher(*simple_car, vehicle_number);
  AddPublisher(*coord_transform, vehicle_number);
  AddBoxcar(coord_transform);
}

template <typename T>
void AutomotiveSimulator<T>::AddTrajectoryCar(
    const Curve2<double>& curve, double speed, double start_time) {
  DRAKE_DEMAND(!started_);
  const int vehicle_number = allocate_vehicle_number();

  auto trajectory_car = builder_->template AddSystem<TrajectoryCar<T>>(
      curve, speed, start_time);
  auto coord_transform =
      builder_->template AddSystem<SimpleCarToEulerFloatingJoint<T>>();

  builder_->Connect(*trajectory_car, *coord_transform);
  AddPublisher(*trajectory_car, vehicle_number);
  AddPublisher(*coord_transform, vehicle_number);
  AddBoxcar(coord_transform);
}

template <typename T>
void AutomotiveSimulator<T>::AddBoxcar(
    const SimpleCarToEulerFloatingJoint<T>* coord_transform) {
  const std::string urdf_filename =
      GetDrakePath() + "/automotive/models/boxcar.urdf";
  const parsers::ModelInstanceIdTable table =
      parsers::urdf::AddModelInstanceFromUrdfFile(
          urdf_filename, rigid_body_tree_.get());
  DRAKE_DEMAND(table.size() == 1);
  const int model_instance_id = table.begin()->second;
  const std::vector<const RigidBody*> bodies =
      rigid_body_tree_->FindModelInstanceBodies(model_instance_id);
  DRAKE_DEMAND(bodies.size() == 1);
  rigid_body_tree_publisher_inputs_.push_back(
      std::make_pair(bodies[0], coord_transform));
}

template <typename T>
void AutomotiveSimulator<T>::AddPublisher(const SimpleCar<T>& system,
                                          int vehicle_number) {
  DRAKE_DEMAND(!started_);
  static const SimpleCarStateTranslator translator;
  auto publisher =
      builder_->template AddSystem<systems::lcm::LcmPublisherSystem>(
          std::to_string(vehicle_number) + "_SIMPLE_CAR_STATE",
          translator, lcm_.get());
  builder_->Connect(system, *publisher);
}

template <typename T>
void AutomotiveSimulator<T>::AddPublisher(const TrajectoryCar<T>& system,
                                          int vehicle_number) {
  DRAKE_DEMAND(!started_);
  static const SimpleCarStateTranslator translator;
  auto publisher =
      builder_->template AddSystem<systems::lcm::LcmPublisherSystem>(
          std::to_string(vehicle_number) + "_SIMPLE_CAR_STATE",
          translator, lcm_.get());
  builder_->Connect(system, *publisher);
}

template <typename T>
void AutomotiveSimulator<T>::AddPublisher(
    const SimpleCarToEulerFloatingJoint<T>& system,
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
void AutomotiveSimulator<T>::Start() {
  DRAKE_DEMAND(!started_);

  if (!rigid_body_tree_publisher_inputs_.empty()) {
    // Arithmetic for RigidBodyTreeLcmPublisher input sizing.  We have systems
    // that output an Euler floating joint state.  We want to mux them together
    // to feed RigidBodyTreeLcmPublisher.  We stack them up in joint order as
    // the position input to the publisher, and then also need to feed zeros
    // for all of the joint velocities.
    const int num_joints = rigid_body_tree_publisher_inputs_.size();
    const int num_ports_into_mux = 2 * num_joints;  // For position + velocity.
    const int num_elements_per_joint =
        EulerFloatingJointStateIndices::kNumCoordinates;

    // Create and cascade a mux and publisher.
    auto multiplexer = builder_->template AddSystem<systems::Multiplexer<T>>(
        std::vector<int>(num_ports_into_mux, num_elements_per_joint));
    auto rigid_body_tree_publisher =
        builder_->template AddSystem<systems::RigidBodyTreeLcmPublisher>(
            *rigid_body_tree_, lcm_.get());
    builder_->Connect(*multiplexer, *rigid_body_tree_publisher);

    // Create a zero-velocity source.
    auto zero_velocity =
        builder_->template AddSystem<systems::ConstantVectorSource>(
            Vector6<T>::Zero().eval());

    // Connect systems that provide joint positions to the mux position inputs.
    // Connect the zero-velocity source to all of the mux velocity inputs.
    for (int input_index = 0; input_index < num_joints; ++input_index) {
      const RigidBody* body{};
      const systems::System<T>* system{};
      std::tie(body, system) = rigid_body_tree_publisher_inputs_[input_index];
      // The 0'th index is the world, so our bodies start at number 1.
      DRAKE_DEMAND(body->get_body_index() == (1 + input_index));
      // Ensure the Publisher inputs correspond to the joints we have.
      DRAKE_DEMAND(body->get_position_start_index() == (
          input_index * num_elements_per_joint));

      builder_->Connect(system->get_output_port(0),
                        multiplexer->get_input_port(input_index));
      builder_->Connect(zero_velocity->get_output_port(),
                        multiplexer->get_input_port(num_joints + input_index));
    }
  }
  rigid_body_tree_->compile();

  diagram_ = builder_->Build();
  simulator_ = std::make_unique<systems::Simulator<T>>(*diagram_);
  lcm_receive_thread_ =
      std::make_unique<systems::lcm::LcmReceiveThread>(lcm_.get());

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

template class DRAKEAUTOMOTIVE_EXPORT AutomotiveSimulator<double>;

}  // namespace automotive
}  // namespace drake
