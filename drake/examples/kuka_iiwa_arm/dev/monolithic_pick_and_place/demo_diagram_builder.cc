#include "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/demo_diagram_builder.h"

#include <memory>

#include "drake/common/drake_path.h"
#include "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/pick_and_place_common.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_lcm.h"
#include "drake/examples/schunk_wsg/schunk_wsg_lcm.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
using systems::DrakeVisualizer;
using systems::DiagramBuilder;
using lcm::DrakeLcm;
using systems::RigidBodyPlant;

namespace examples {
using schunk_wsg::SchunkWsgTrajectoryGenerator;
using schunk_wsg::SchunkWsgStatusSender;

namespace kuka_iiwa_arm {
namespace monolithic_pick_and_place {

template <typename T>
StateMachineAndPrimitives<T>::StateMachineAndPrimitives(
    const RigidBodyTree<T>& iiwa_tree, const double iiwa_action_primitive_rate,
    const double wsg_action_primitive_rate) {
  this->set_name("StateMachineAndPrimitives");
  DiagramBuilder<T> builder;

  iiwa_move_ = builder.template AddSystem<IiwaMove>(iiwa_tree,
                                                    iiwa_action_primitive_rate);
  iiwa_move_->set_name("IiwaMove");

  gripper_action_ =
      builder.template AddSystem<GripperAction>(wsg_action_primitive_rate);
  gripper_action_->set_name("GripperAction");

  // TODO(naveenoid): This needs to take robot base orientation into account
  // eventually.
  Isometry3<T> iiwa_base = Isometry3<T>::Identity();
  iiwa_base.translation() = kRobotBase;

  pick_and_place_state_machine_ =
      builder.template AddSystem<PickAndPlaceStateMachineSystem>(iiwa_base);
  pick_and_place_state_machine_->set_name("PickAndPlaceStateMachine");

  input_port_iiwa_robot_state_t_ = builder.ExportInput(
      pick_and_place_state_machine_->get_input_port_iiwa_state());

  input_port_box_robot_state_t_ = builder.ExportInput(
      pick_and_place_state_machine_->get_input_port_box_state());

  input_port_wsg_status_ = builder.ExportInput(
      pick_and_place_state_machine_->get_input_port_wsg_status());

  builder.Connect(pick_and_place_state_machine_->get_output_port_iiwa_action(),
                  iiwa_move_->get_primitive_input_port());
  builder.Connect(pick_and_place_state_machine_->get_output_port_wsg_action(),
                  gripper_action_->get_primitive_input_port());

  builder.Connect(
      iiwa_move_->get_status_output_port(),
      pick_and_place_state_machine_->get_input_port_iiwa_action_status());

  builder.Connect(
      gripper_action_->get_status_output_port(),
      pick_and_place_state_machine_->get_input_port_wsg_action_status());

  output_port_iiwa_command_ =
      builder.ExportOutput(iiwa_move_->get_robot_plan_output_port());
  output_port_wsg_command_ =
      builder.ExportOutput(gripper_action_->get_robot_plan_output_port());

  builder.BuildInto(this);
}
template class StateMachineAndPrimitives<double>;

template <typename T>
IiwaWsgPlantGeneratorsEstimatorsAndVisualizer<
    T>::IiwaWsgPlantGeneratorsEstimatorsAndVisualizer(DrakeLcm* lcm,
                                                      const double
                                                          update_interval) {
  this->set_name("IiwaWsgPlantGeneratorsEstimatorsAndVisualizer");

  ModelInstanceInfo<double> iiwa_instance, wsg_instance, box_instance;

  std::unique_ptr<systems::RigidBodyPlant<double>> model_ptr =
      BuildCombinedPlant<double>(&iiwa_instance, &wsg_instance, &box_instance);

  DiagramBuilder<T> builder;
  plant_ = builder.template AddSystem<IiwaAndWsgPlantWithStateEstimator<T>>(
      std::move(model_ptr), iiwa_instance, wsg_instance, box_instance);
  plant_->set_name("plant");

  drake_visualizer_ = builder.template AddSystem<DrakeVisualizer>(
      plant_->get_plant().get_rigid_body_tree(), lcm);
  drake_visualizer_->set_name("drake_visualizer");

  builder.Connect(plant_->get_output_port_plant_state(),
                  drake_visualizer_->get_input_port(0));

  iiwa_trajectory_generator_ =
      builder.template AddSystem<IiwaPlanSource>(
          drake::GetDrakePath() + kIiwaUrdf, update_interval);
  iiwa_trajectory_generator_->set_name("iiwa_trajectory_generator");

  builder.Connect(plant_->get_output_port_iiwa_state(),
                  iiwa_trajectory_generator_->get_state_input_port());
  builder.Connect(
      iiwa_trajectory_generator_->get_state_output_port(),
      plant_->get_input_port_iiwa_state_command());
  builder.Connect(
      iiwa_trajectory_generator_->get_acceleration_output_port(),
      plant_->get_input_port_iiwa_acceleration_command());

  input_port_iiwa_plan_ =
      builder.ExportInput(iiwa_trajectory_generator_->get_plan_input_port());

  wsg_trajectory_generator_ =
      builder.template AddSystem<SchunkWsgTrajectoryGenerator>(
          plant_->get_output_port_wsg_state().size(), 0);
  wsg_trajectory_generator_->set_name("wsg_trajectory_generator");

  builder.Connect(plant_->get_output_port_wsg_state(),
                  wsg_trajectory_generator_->get_state_input_port());
  builder.Connect(wsg_trajectory_generator_->get_output_port(0),
                  plant_->get_input_port_wsg_command());
  input_port_wsg_plan_ =
      builder.ExportInput(wsg_trajectory_generator_->get_command_input_port());

  output_port_iiwa_robot_state_msg_ =
      builder.ExportOutput(plant_->get_output_port_iiwa_robot_state_msg());
  output_port_box_robot_state_msg_ =
      builder.ExportOutput(plant_->get_output_port_box_robot_state_msg());

  // Sets up a WSG Status sender.
  wsg_status_sender_ = builder.template AddSystem<SchunkWsgStatusSender>(
      plant_->get_output_port_wsg_state().size(), 0, 0);
  wsg_status_sender_->set_name("wsg_status_sender");

  builder.Connect(plant_->get_output_port_wsg_state(),
                  wsg_status_sender_->get_input_port(0));
  output_port_wsg_status_ =
      builder.ExportOutput(wsg_status_sender_->get_output_port(0));

  builder.BuildInto(this);
}

template <typename T>
void IiwaWsgPlantGeneratorsEstimatorsAndVisualizer<T>::InitializeIiwaPlan(
    const VectorX<T>& q0, systems::Context<T>* context) const {
  auto plan_source_context =
      this->GetMutableSubsystemContext(
          context, iiwa_trajectory_generator_);
  iiwa_trajectory_generator_->Initialize(
      context->get_time(), q0,
      plan_source_context->get_mutable_state());
}


template class IiwaWsgPlantGeneratorsEstimatorsAndVisualizer<double>;

}  // namespace monolithic_pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
