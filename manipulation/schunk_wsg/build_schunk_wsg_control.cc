#include "drake/manipulation/schunk_wsg/build_schunk_wsg_control.h"

#include "drake/lcmt_schunk_wsg_command.hpp"
#include "drake/lcmt_schunk_wsg_status.hpp"
#include "drake/manipulation/schunk_wsg/schunk_wsg_constants.h"
#include "drake/manipulation/schunk_wsg/schunk_wsg_controller.h"
#include "drake/manipulation/schunk_wsg/schunk_wsg_lcm.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"

namespace drake {
namespace manipulation {
namespace schunk_wsg {

using lcm::DrakeLcmInterface;
using multibody::ModelInstanceIndex;
using multibody::MultibodyPlant;
using systems::lcm::LcmPublisherSystem;
using systems::lcm::LcmSubscriberSystem;

void BuildSchunkWsgControl(const MultibodyPlant<double>& plant,
                           const ModelInstanceIndex wsg_instance,
                           DrakeLcmInterface* lcm,
                           systems::DiagramBuilder<double>* builder,
                           const std::optional<Eigen::Vector3d>& pid_gains) {
  // Create gripper command subscriber.
  auto wsg_command_sub =
      builder->AddSystem(LcmSubscriberSystem::Make<lcmt_schunk_wsg_command>(
          "SCHUNK_WSG_COMMAND", lcm));
  wsg_command_sub->set_name(plant.GetModelInstanceName(wsg_instance) +
                            "_wsg_command_subscriber");
  Eigen::Vector3d desired_pid_gains =
      pid_gains.value_or(Eigen::Vector3d(7200.0, 0.0, 5.0));
  auto wsg_controller = builder->AddSystem<SchunkWsgController>(
      desired_pid_gains(0), desired_pid_gains(1), desired_pid_gains(2));
  builder->Connect(wsg_command_sub->get_output_port(),
                   wsg_controller->GetInputPort("command_message"));
  builder->Connect(wsg_controller->GetOutputPort("force"),
                   plant.get_actuation_input_port(wsg_instance));

  // Create gripper status publisher.
  auto wsg_status_pub =
      builder->AddSystem(LcmPublisherSystem::Make<lcmt_schunk_wsg_status>(
          "SCHUNK_WSG_STATUS", lcm, kSchunkWsgLcmStatusPeriod));
  wsg_status_pub->set_name(plant.GetModelInstanceName(wsg_instance) +
                           "_wsg_status_publisher");
  auto wsg_status_sender = builder->AddSystem<SchunkWsgStatusSender>();
  builder->Connect(*wsg_status_sender, *wsg_status_pub);

  auto wsg_mbp_state_to_wsg_state =
      builder->AddSystem(MakeMultibodyStateToWsgStateSystem<double>());
  builder->Connect(plant.get_state_output_port(wsg_instance),
                   wsg_mbp_state_to_wsg_state->get_input_port());
  builder->Connect(wsg_mbp_state_to_wsg_state->get_output_port(),
                   wsg_status_sender->get_state_input_port());

  // TODO(sam.creasey) This should be the measured joint torque according to
  // the plant.
  auto mbp_force_to_wsg_force =
      builder->AddSystem(MakeMultibodyForceToWsgForceSystem<double>());
  builder->Connect(
      plant.get_generalized_contact_forces_output_port(wsg_instance),
      mbp_force_to_wsg_force->get_input_port());
  builder->Connect(mbp_force_to_wsg_force->get_output_port(),
                   wsg_status_sender->get_force_input_port());
  builder->Connect(plant.get_state_output_port(wsg_instance),
                   wsg_controller->GetInputPort("state"));
}

}  // namespace schunk_wsg
}  // namespace manipulation
}  // namespace drake
