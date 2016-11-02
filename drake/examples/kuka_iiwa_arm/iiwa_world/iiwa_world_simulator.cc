#include <gflags/gflags.h>

#include "drake/common/drake_path.h"
#include "drake/common/text_logging_gflags.h"

#include "drake/examples/kuka_iiwa_arm/iiwa_world/iiwa_world_simulator.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/common/drake_export.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/primitives/constant_vector_source.h"
#include "drake/systems/framework/primitives/multiplexer.h"

#include "drake/systems/plants/parser_model_instance_id_table.h"
#include "drake/systems/plants/parser_sdf.h"
#include "drake/systems/plants/parser_urdf.h"
#include "drake/systems/plants/rigid_body_plant/drake_visualizer.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

template <typename T>
IiwaWorldSimulator<T>::IiwaWorldSimulator()
    : IiwaWorldSimulator(std::make_unique<lcm::DrakeLcm>()) {}

template <typename T>
IiwaWorldSimulator<T>::IiwaWorldSimulator(
    std::unique_ptr<lcm::DrakeLcmInterface> lcm) :
    lcm_(std::move(lcm)) {}

template <typename T>
IiwaWorldSimulator<T>::~IiwaWorldSimulator() {
  lcm_.reset();
}

template <typename T>
int IiwaWorldSimulator<T>::AddIiwaArm(bool with_gripper) {

  DRAKE_DEMAND(!started_);

  parsers::ModelInstanceIdTable table =
      drake::parsers::urdf::AddModelInstanceFromUrdfFile(
          drake::GetDrakePath() + "/examples/kuka_iiwa_arm/urdf/iiwa14.urdf",
          systems::plants::joints::kFixed, nullptr /* weld to frame */, rigid_body_tree_.get());

  AddGround(rigid_body_tree_.get());

//  const int object_number = allocate_object_number();

//  const parsers::ModelInstanceIdTable table =
//      parsers::sdf::AddModelInstancesFromSdfFileInWorldFrame(
//          sdf_filename, kRollPitchYaw, rigid_body_tree_.get());

  const int model_instance_id = table.begin()->second;
  const std::vector<const RigidBody*> bodies =
      rigid_body_tree_->FindModelInstanceBodies(model_instance_id);
//  rigid_body_tree_publisher_inputs_.push_back(
//      std::make_pair(model_instance_id, coord_transform));
  return model_instance_id;
}

template <typename T>
void IiwaWorldSimulator<T>::Build() {
  DRAKE_DEMAND(!started_);

  std::cout<<"Trying to build the system\n";
  //if (!drake_visualizer_inputs_.empty()) {
    // Arithmetic for DrakeVisualizer input sizing.  We have systems that output
    // an Euler floating joint state.  We want to mux them together to feed
    // DrakeVisualizer.  We stack them up in joint order as the position input
    // to the publisher, and then also need to feed zeros for all of the joint
    // velocities.

    std::cout<<"Building the diagram\n";

    const int num_joints = drake_visualizer_inputs_.size();
    const int num_ports_into_mux =  1;// 2 * num_joints;  // For position + velocity.
    std::cout<<"Num joints :"<<num_joints<<"\n";
    const int num_elements_per_joint = 6; //6 DoF.
        //EulerFloatingJointStateIndices::kNumCoordinates;

    // Create and cascade a mux and publisher.
    auto multiplexer = builder_->template AddSystem<systems::Multiplexer<T>>(
        std::vector<int>());
  // num_ports_into_mux, num_elements_per_joint));
    auto rigid_body_tree_publisher =
        builder_->template AddSystem<systems::DrakeVisualizer>(
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
      std::tie(body, system) = drake_visualizer_inputs_[input_index];
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
//  }
  rigid_body_tree_->compile();

  diagram_ = builder_->Build();
  simulator_ = std::make_unique<systems::Simulator<T>>(*diagram_);
  lcm_->StartReceiveThread();

  simulator_->Initialize();

  started_ = true;
}

template <typename T>
const systems::System<T>& IiwaWorldSimulator<T>::GetDiagramSystemByName(
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
void IiwaWorldSimulator<T>::StepBy(const T& time_step) {
  const T time = simulator_->get_context().get_time();
  SPDLOG_TRACE(drake::log(), "Time is now {}", time);
  simulator_->StepTo(time + time_step);
}

template <typename T>
int IiwaWorldSimulator<T>::allocate_object_number() {
  DRAKE_DEMAND(!started_);
  return next_object_number_++;
}


template class DRAKE_EXPORT IiwaWorldSimulator<double>;
}
}
}