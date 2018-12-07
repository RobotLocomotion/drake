#include "drake/examples/kuka_iiwa_arm/dev/pick_and_place/lcm_plant.h"

#include <map>
#include <memory>
#include <string>
#include <utility>

#include "drake/examples/kuka_iiwa_arm/dev/pick_and_place/pick_and_place_simulation_helpers.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_lcm.h"
#include "drake/manipulation/schunk_wsg/schunk_wsg_constants.h"
#include "drake/manipulation/schunk_wsg/schunk_wsg_controller.h"
#include "drake/manipulation/schunk_wsg/schunk_wsg_lcm.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace pick_and_place {

using manipulation::schunk_wsg::MakeMultibodyForceToWsgForceSystem;
using manipulation::schunk_wsg::MakeMultibodyStateToWsgStateSystem;
using manipulation::schunk_wsg::SchunkWsgController;
using manipulation::schunk_wsg::SchunkWsgStatusSender;
using manipulation::util::ModelInstanceInfo;
using systems::ConstantVectorSource;

LcmPlant::LcmPlant(
    const SimulatedPlantConfiguration& plant_configuration,
    const OptitrackConfiguration& optitrack_configuration) {
  DRAKE_THROW_UNLESS(plant_configuration.robot_models.size() ==
                     optitrack_configuration.robot_base_optitrack_info.size());
  DRAKE_THROW_UNLESS(plant_configuration.object_models.size() ==
                     optitrack_configuration.object_optitrack_info.size());
  DRAKE_THROW_UNLESS(plant_configuration.table_models.size() ==
                     optitrack_configuration.table_optitrack_info.size());
  systems::DiagramBuilder<double> builder;
  std::vector<ModelInstanceInfo<double>> iiwa_instances, wsg_instances,
      object_instances, table_instances;
  std::unique_ptr<systems::RigidBodyPlant<double>> model_ptr =
      BuildPickAndPlacePlant(
          plant_configuration, &iiwa_instances, &wsg_instances,
          &object_instances, &table_instances);
  model_ptr->set_default_compliant_material(
      plant_configuration.default_contact_material);
  model_ptr->set_contact_model_parameters(
      plant_configuration.contact_model_parameters);

  iiwa_and_wsg_plant_ =
      builder.AddSystem<IiwaAndWsgPlantWithStateEstimator<double>>(
          std::move(model_ptr), iiwa_instances, wsg_instances,
          object_instances);
  iiwa_and_wsg_plant_->set_name("iiwa_and_wsg_plant_");

  const RigidBodyTree<double>& tree = iiwa_and_wsg_plant_->get_tree();
  const systems::OutputPort<double>& optitrack_lcm_output_port =
      AddOptitrackComponents(
          optitrack_configuration, tree,
          iiwa_instances, object_instances,
          table_instances,
          iiwa_and_wsg_plant_->get_output_port_kinematics_results(),
          &builder);

  // Export Optitrack output port.
  output_port_optitrack_frame_ =
      builder.ExportOutput(optitrack_lcm_output_port);

  // Export contact results output port.
  output_port_contact_results_ = builder.ExportOutput(
      iiwa_and_wsg_plant_->get_output_port_contact_results());

  // Export plant state output port.
  output_port_plant_state_ =
      builder.ExportOutput(iiwa_and_wsg_plant_->get_output_port_plant_state());

  std::vector<IiwaCommandReceiver*> iiwa_command_receivers;
  const int kNumIiwas = plant_configuration.robot_poses.size();
  for (int i = 0; i < kNumIiwas; ++i) {
    const std::string suffix{"_" + std::to_string(i)};
    auto iiwa_command_receiver =
        builder.AddSystem<IiwaCommandReceiver>(kIiwaArmNumJoints);
    iiwa_command_receivers.push_back(iiwa_command_receiver);
    iiwa_command_receiver->set_name("iiwa_command_receiver" + suffix);

    // Export iiwa command input port.
    input_port_iiwa_command_.push_back(builder.ExportInput(
        iiwa_command_receiver->GetInputPort("command_message")));
    builder.Connect(iiwa_command_receiver->get_commanded_state_output_port(),
                    iiwa_and_wsg_plant_->get_input_port_iiwa_state_command(i));

    // lcmt_iiwa_command does not include a reference acceleration, so use a
    // zero constant source for the controller's acceleration input.
    auto zero_feedforward_acceleration =
        builder.AddSystem<ConstantVectorSource<double>>(VectorX<double>::Zero(
            iiwa_and_wsg_plant_->get_input_port_iiwa_acceleration_command()
                .size()));
    builder.Connect(
        zero_feedforward_acceleration->get_output_port(),
        iiwa_and_wsg_plant_->get_input_port_iiwa_acceleration_command(i));

    auto iiwa_status_sender =
        builder.AddSystem<IiwaStatusSender>(kIiwaArmNumJoints);
    iiwa_status_sender->set_name("iiwa_status_sender" + suffix);

    builder.Connect(iiwa_and_wsg_plant_->get_output_port_iiwa_state(i),
                    iiwa_status_sender->get_state_input_port());
    builder.Connect(iiwa_command_receiver->get_output_port(0),
                    iiwa_status_sender->get_command_input_port());
    builder.Connect(iiwa_and_wsg_plant_->get_output_port_computed_torque(),
                    iiwa_status_sender->get_commanded_torque_input_port());

    // Export iiwa status output port.
    output_port_iiwa_status_.push_back(
        builder.ExportOutput(iiwa_status_sender->get_output_port(0)));

    auto wsg_controller = builder.AddSystem<SchunkWsgController>();
    wsg_controller->set_name("wsg_controller" + suffix);
    builder.Connect(iiwa_and_wsg_plant_->get_output_port_wsg_state(i),
                    wsg_controller->GetInputPort("state"));
    builder.Connect(wsg_controller->GetOutputPort("force"),
                    iiwa_and_wsg_plant_->get_input_port_wsg_command(i));

    auto wsg_status_sender = builder.AddSystem<SchunkWsgStatusSender>();
    auto mbp_state_to_wsg_state =
        builder.AddSystem(MakeMultibodyStateToWsgStateSystem<double>());
    wsg_status_sender->set_name("wsg_status_sender" + suffix);
    builder.Connect(iiwa_and_wsg_plant_->get_output_port_wsg_state(i),
                    mbp_state_to_wsg_state->get_input_port());
    builder.Connect(mbp_state_to_wsg_state->get_output_port(),
                    wsg_status_sender->get_state_input_port());
    auto mbp_force_to_wsg_force =
        builder.AddSystem(MakeMultibodyForceToWsgForceSystem<double>());
    builder.Connect(iiwa_and_wsg_plant_->get_output_port_wsg_measured_torque(i),
                    mbp_force_to_wsg_force->get_input_port());
    builder.Connect(mbp_force_to_wsg_force->get_output_port(),
                    wsg_status_sender->get_force_input_port());

    // Export wsg status output port.
    output_port_wsg_status_.push_back(
        builder.ExportOutput(wsg_status_sender->get_output_port(0)));

    // Export WSG command input port.
    input_port_wsg_command_.push_back(
        builder.ExportInput(wsg_controller->GetInputPort("command_message")));
  }

  // Build the system.
  builder.BuildInto(this);
}

}  // namespace pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
