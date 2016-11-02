#include "drake/automotive/automotive_simulator.h"

#include "drake/automotive/gen/driving_command_translator.h"
#include "drake/automotive/gen/euler_floating_joint_state_translator.h"
#include "drake/automotive/gen/simple_car_state_translator.h"
#include "drake/automotive/simple_car.h"
#include "drake/automotive/simple_car_to_euler_floating_joint.h"
#include "drake/automotive/trajectory_car.h"
#include "drake/common/drake_export.h"
#include "drake/common/drake_path.h"
#include "drake/common/drake_throw.h"
#include "drake/common/text_logging.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/primitives/constant_vector_source.h"
#include "drake/systems/framework/primitives/multiplexer.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/plants/joints/floating_base_types.h"
#include "drake/systems/plants/parser_model_instance_id_table.h"
#include "drake/systems/plants/parser_sdf.h"
#include "drake/systems/plants/parser_urdf.h"
#include "drake/systems/plants/rigid_body_plant/drake_visualizer.h"

namespace drake {
namespace automotive {

using drake::systems::plants::joints::kRollPitchYaw;

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
  // TODO(liang.fok) Allow model to be client selectable.
  AddSdfModel(GetDrakePath() + "/automotive/models/prius/prius_with_lidar.sdf",
              coord_transform);
}

template <typename T>
void AutomotiveSimulator<T>::AddTrajectoryCar(const Curve2<double>& curve,
                                              double speed, double start_time) {
  DRAKE_DEMAND(!started_);
  const int vehicle_number = allocate_vehicle_number();

  auto trajectory_car =
      builder_->template AddSystem<TrajectoryCar<T>>(curve, speed, start_time);
  auto coord_transform =
      builder_->template AddSystem<SimpleCarToEulerFloatingJoint<T>>();

  builder_->Connect(*trajectory_car, *coord_transform);
  AddPublisher(*trajectory_car, vehicle_number);
  AddPublisher(*coord_transform, vehicle_number);
  // TODO(liang.fok) Allow model to be client selectable.
  AddSdfModel(GetDrakePath() + "/automotive/models/prius/prius_with_lidar.sdf",
              coord_transform);
}

template <typename T>
void AutomotiveSimulator<T>::AddSdfModel(
    const std::string& sdf_filename,
    const SimpleCarToEulerFloatingJoint<T>* coord_transform) {
  const parsers::ModelInstanceIdTable table =
      parsers::sdf::AddModelInstancesFromSdfFileInWorldFrame(
          sdf_filename, kRollPitchYaw, rigid_body_tree_.get());

  // TODO(liang.fok): Add support for SDF files containing more than one model.
  DRAKE_DEMAND(table.size() == 1);

  const int model_instance_id = table.begin()->second;
  rigid_body_tree_publisher_inputs_.push_back(
      std::make_pair(model_instance_id, coord_transform));
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
  builder_->Connect(system, *publisher);
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
    const std::vector<const RigidBody*> bodies =
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
void AutomotiveSimulator<T>::Start() {
  DRAKE_DEMAND(!started_);
  // By this time, all model instances should have been added to the tree. Thus,
  // it should be safe to compile the tree.
  rigid_body_tree_->compile();

  ConnectJointStateSourcesToVisualizer();

  diagram_ = builder_->Build();
  simulator_ = std::make_unique<systems::Simulator<T>>(*diagram_);
  lcm_->StartReceiveThread();

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

template class DRAKE_EXPORT AutomotiveSimulator<double>;

}  // namespace automotive
}  // namespace drake
