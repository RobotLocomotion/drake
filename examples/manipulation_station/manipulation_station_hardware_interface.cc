#include "drake/examples/manipulation_station/manipulation_station_hardware_interface.h"

#include <utility>

#include "drake/common/find_resource.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_iiwa_command.hpp"
#include "drake/lcmt_iiwa_status.hpp"
#include "drake/lcmt_image_array.hpp"
#include "drake/lcmt_schunk_wsg_command.hpp"
#include "drake/lcmt_schunk_wsg_status.hpp"
#include "drake/manipulation/kuka_iiwa/iiwa_command_sender.h"
#include "drake/manipulation/kuka_iiwa/iiwa_status_receiver.h"
#include "drake/manipulation/schunk_wsg/schunk_wsg_lcm.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/primitives/pass_through.h"
#include "drake/systems/sensors/lcm_image_array_to_images.h"

namespace drake {
namespace examples {
namespace manipulation_station {

using Eigen::Vector3d;
using multibody::MultibodyPlant;
using multibody::Parser;

// TODO(russt): Consider taking DrakeLcmInterface as an argument instead of
// (only) constructing one internally.
ManipulationStationHardwareInterface::ManipulationStationHardwareInterface(
    std::vector<std::string> camera_names)
    : owned_controller_plant_(std::make_unique<MultibodyPlant<double>>(0.0)),
      owned_lcm_(new lcm::DrakeLcm()),
      camera_names_(std::move(camera_names)) {
  systems::DiagramBuilder<double> builder;

  auto lcm = builder.AddSystem<systems::lcm::LcmInterfaceSystem>(
      owned_lcm_.get());

  // Publish IIWA command.
  auto iiwa_command_sender =
      builder.AddSystem<manipulation::kuka_iiwa::IiwaCommandSender>();
  auto iiwa_command_publisher = builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<drake::lcmt_iiwa_command>(
          "IIWA_COMMAND", lcm, 0.005
          /* publish period: IIWA driver won't respond faster than 200Hz */));
  builder.ExportInput(iiwa_command_sender->get_position_input_port(),
                      "iiwa_position");
  builder.ExportInput(iiwa_command_sender->get_torque_input_port(),
                      "iiwa_feedforward_torque");
  builder.Connect(iiwa_command_sender->get_output_port(),
                  iiwa_command_publisher->get_input_port());

  // Receive IIWA status and populate the output ports.
  auto iiwa_status_receiver =
      builder.AddSystem<manipulation::kuka_iiwa::IiwaStatusReceiver>();
  iiwa_status_subscriber_ = builder.AddSystem(
      systems::lcm::LcmSubscriberSystem::Make<drake::lcmt_iiwa_status>(
          "IIWA_STATUS", lcm));

  builder.ExportOutput(
      iiwa_status_receiver->get_position_commanded_output_port(),
      "iiwa_position_commanded");
  builder.ExportOutput(
      iiwa_status_receiver->get_position_measured_output_port(),
      "iiwa_position_measured");
  builder.ExportOutput(
      iiwa_status_receiver->get_velocity_estimated_output_port(),
      "iiwa_velocity_estimated");
  builder.ExportOutput(iiwa_status_receiver->get_torque_commanded_output_port(),
                       "iiwa_torque_commanded");
  builder.ExportOutput(iiwa_status_receiver->get_torque_measured_output_port(),
                       "iiwa_torque_measured");
  builder.ExportOutput(iiwa_status_receiver->get_torque_external_output_port(),
                       "iiwa_torque_external");
  builder.Connect(iiwa_status_subscriber_->get_output_port(),
                  iiwa_status_receiver->get_input_port());

  // Publish WSG command.
  auto wsg_command_sender =
      builder.AddSystem<manipulation::schunk_wsg::SchunkWsgCommandSender>();
  auto wsg_command_publisher = builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<drake::lcmt_schunk_wsg_command>(
          "SCHUNK_WSG_COMMAND", lcm, 0.05
          /* publish period: Schunk driver won't respond faster than 20Hz */));
  builder.ExportInput(wsg_command_sender->get_position_input_port(),
                      "wsg_position");
  builder.ExportInput(wsg_command_sender->get_force_limit_input_port(),
                      "wsg_force_limit");
  builder.Connect(wsg_command_sender->get_output_port(0),
                  wsg_command_publisher->get_input_port());

  // Receive WSG status and populate the output ports.
  auto wsg_status_receiver =
      builder.AddSystem<manipulation::schunk_wsg::SchunkWsgStatusReceiver>();
  wsg_status_subscriber_ = builder.AddSystem(
      systems::lcm::LcmSubscriberSystem::Make<drake::lcmt_schunk_wsg_status>(
          "SCHUNK_WSG_STATUS", lcm));
  builder.ExportOutput(wsg_status_receiver->get_state_output_port(),
                       "wsg_state_measured");
  builder.ExportOutput(wsg_status_receiver->get_force_output_port(),
                       "wsg_force_measured");
  builder.Connect(wsg_status_subscriber_->get_output_port(),
                  wsg_status_receiver->get_input_port(0));

  for (const std::string& name : camera_names_) {
    auto camera_subscriber = builder.AddSystem(
        systems::lcm::LcmSubscriberSystem::Make<lcmt_image_array>(
            "DRAKE_RGBD_CAMERA_IMAGES_" + name, lcm));
    auto array_to_images =
        builder.AddSystem<systems::sensors::LcmImageArrayToImages>();
    builder.Connect(camera_subscriber->get_output_port(),
                    array_to_images->image_array_t_input_port());
    builder.ExportOutput(
        array_to_images->color_image_output_port(),
        "camera_" + name + "_rgb_image");
    builder.ExportOutput(
        array_to_images->depth_image_output_port(),
        "camera_" + name + "_depth_image");
    camera_subscribers_.push_back(camera_subscriber);
  }

  builder.BuildInto(this);
  this->set_name("manipulation_station_hardware_interface");

  // Build the controller's version of the plant, which only contains the
  // IIWA and the equivalent inertia of the gripper.
  const std::string iiwa_sdf_path = FindResourceOrThrow(
      "drake/manipulation/models/iiwa_description/sdf/iiwa14_no_collision.sdf");
  Parser parser(owned_controller_plant_.get());
  iiwa_model_instance_ = parser.AddModelFromFile(iiwa_sdf_path, "iiwa");

  // TODO(russt): Provide API for changing the base coordinates of the plant.
  owned_controller_plant_->WeldFrames(owned_controller_plant_->world_frame(),
                                      owned_controller_plant_->GetFrameByName(
                                          "iiwa_link_0", iiwa_model_instance_),
                                      math::RigidTransformd::Identity());
  owned_controller_plant_->Finalize();
}

void ManipulationStationHardwareInterface::Connect(bool wait_for_cameras) {
  drake::lcm::DrakeLcmInterface* const lcm = owned_lcm_.get();
  auto wait_for_new_message = [lcm](const auto& lcm_sub) {
    std::cout << "Waiting for " << lcm_sub.get_channel_name()
              << " message..." << std::flush;
    const int orig_count = lcm_sub.GetInternalMessageCount();
    LcmHandleSubscriptionsUntil(lcm, [&]() {
        return lcm_sub.GetInternalMessageCount() > orig_count;
      }, 10 /* timeout_millis */);
    std::cout << "Received!" << std::endl;
  };

  wait_for_new_message(*iiwa_status_subscriber_);
  wait_for_new_message(*wsg_status_subscriber_);
  if (wait_for_cameras) {
    for (const auto* sub : camera_subscribers_) {
      wait_for_new_message(*sub);
    }
  }
}

int ManipulationStationHardwareInterface::num_iiwa_joints() const {
  DRAKE_DEMAND(iiwa_model_instance_.is_valid());
  return owned_controller_plant_->num_positions(iiwa_model_instance_);
}

}  // namespace manipulation_station
}  // namespace examples
}  // namespace drake
