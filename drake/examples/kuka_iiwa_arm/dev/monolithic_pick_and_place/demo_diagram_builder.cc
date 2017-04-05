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
namespace pick_and_place {

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

  drake_visualizer_ = builder.template AddSystem<DrakeVisualizer>(
      plant_->get_plant().get_rigid_body_tree(), lcm);

  builder.Connect(plant_->get_output_port_plant_state(),
                  drake_visualizer_->get_input_port(0));

  iiwa_trajectory_generator_ =
      builder.template AddSystem<IiwaStateFeedbackPlanSource>(
          drake::GetDrakePath() + kIiwaUrdf, update_interval);
  builder.Connect(plant_->get_output_port_iiwa_state(),
                  iiwa_trajectory_generator_->get_input_port_state());
  builder.Connect(
      iiwa_trajectory_generator_->get_output_port_state_trajectory(),
      plant_->get_input_port_iiwa_state_command());
  builder.Connect(
      iiwa_trajectory_generator_->get_output_port_acceleration_trajectory(),
      plant_->get_input_port_iiwa_acceleration_command());

  input_port_iiwa_plan_ =
      builder.ExportInput(iiwa_trajectory_generator_->get_input_port_plan());

  wsg_trajectory_generator_ =
      builder.template AddSystem<SchunkWsgTrajectoryGenerator>(
          plant_->get_output_port_wsg_state().size(), 0);
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

  pass_through_wsg_state_ = builder.template AddSystem<PassThrough<T>>(
      plant_->get_output_port_wsg_state().size());

  builder.Connect(plant_->get_output_port_wsg_state(),
                  pass_through_wsg_state_->get_input_port());

  builder.Connect(plant_->get_output_port_wsg_state(),
                  wsg_status_sender_->get_input_port(0));
  output_port_wsg_status_ =
      builder.ExportOutput(wsg_status_sender_->get_output_port(0));

  builder.BuildInto(this);
}
template class IiwaWsgPlantGeneratorsEstimatorsAndVisualizer<double>;

}  // namespace pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
