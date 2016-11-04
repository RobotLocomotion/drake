#include <gflags/gflags.h>

#include "drake/common/drake_path.h"
#include "drake/common/text_logging_gflags.h"

#include "drake/examples/kuka_iiwa_arm/iiwa_world/iiwa_world_simulator.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/systems/plants/rigid_body_plant/rigid_body_plant.h"
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

using lcm::DrakeLcm;
using systems::ConstantVectorSource;
using systems::Context;
using systems::ContinuousState;
using systems::Diagram;
using systems::DiagramBuilder;
using systems::DrakeVisualizer;
using systems::RigidBodyPlant;
using systems::Simulator;
using systems::VectorBase;
using systems::plants::joints::kFixed;


using parsers::ModelInstanceIdTable;
using parsers::urdf::AddModelInstanceFromUrdfFile;

namespace examples {
namespace kuka_iiwa_arm {

template <typename T>
IiwaWorldSimulator<T>::IiwaWorldSimulator() {}
//    : IiwaWorldSimulator(std::make_unique<lcm::DrakeLcm>()) {}

//template <typename T>
//IiwaWorldSimulator<T>::IiwaWorldSimulator(
//    std::unique_ptr<lcm::DrakeLcmInterface> lcm) :
//    lcm_(std::move(lcm)) {}

template <typename T>
IiwaWorldSimulator<T>::~IiwaWorldSimulator() {
}

template <typename T>
int IiwaWorldSimulator<T>::AddIiwaArm() {

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
int IiwaWorldSimulator<T>::AddObject() {

  DRAKE_DEMAND(!started_);


  parsers::ModelInstanceIdTable table =
      drake::parsers::urdf::AddModelInstanceFromUrdfFile(
          drake::GetDrakePath() + "/examples/kuka_iiwa_arm/models/desk/transcendesk55inch.sdf.urdf",
          systems::plants::joints::kFixed, nullptr /* weld to frame */, rigid_body_tree_.get());

//  AddGround(rigid_body_tree_.get());

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
  rigid_body_tree_->compile();


  std::cout<<"Building the diagram\n";

//  auto plant = builder_.template AddSystem<systems::RigidBodyPlant<T>>();
  auto plant =
  builder_->template AddSystem<systems::RigidBodyPlant<T>>(move(rigid_body_tree_));
    plant->set_contact_parameters(3000 /* penetration stiffness */,
                                 0 /* penetration damping */,
                                   1.0 /* friction coefficient */);


  // Feed in constant inputs of zero into the RigidBodyPlant.
  VectorX<T> constant_value(plant->get_input_size());

  std::cout<< " Input size : "<< plant->get_input_size();
  constant_value.setZero();
  auto const_source_ = builder_->template AddSystem<ConstantVectorSource<T>>(
        constant_value);

    // Creates and adds LCM publisher for visualization.
  auto viz_publisher_ = builder_->template AddSystem<DrakeVisualizer>(
     plant->get_rigid_body_tree(), &lcm_);

    // Connects the constant source output port to the RigidBodyPlant's input
    // port. This effectively results in the robot being uncontrolled.
  builder_->Connect(const_source_->get_output_port(),
                    plant->get_input_port(0));

    // Connects to publisher for visualization.
  builder_->Connect(plant->get_output_port(0),
                    viz_publisher_->get_input_port(0));

  builder_->ExportOutput(plant->get_output_port(0));


  diagram_ = builder_->Build();
  simulator_ = std::make_unique<systems::Simulator<T>>(*diagram_);
//  lcm_->StartReceiveThread();
  Context<T>* plant_context =
      diagram_->GetMutableSubsystemContext(simulator_->get_mutable_context(), plant);
  plant->SetZeroConfiguration(plant_context);

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