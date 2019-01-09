#include "drake/examples/manipulation_station/manipulation_station_hardware_interface.h"

#include <utility>

#include "robotlocomotion/image_array_t.hpp"

#include "drake/common/find_resource.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_lcm.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_iiwa_command.hpp"
#include "drake/lcmt_iiwa_status.hpp"
#include "drake/lcmt_schunk_wsg_command.hpp"
#include "drake/lcmt_schunk_wsg_status.hpp"
#include "drake/manipulation/schunk_wsg/schunk_wsg_lcm.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/primitives/pass_through.h"
#include "drake/systems/sensors/lcm_image_array_to_images.h"

namespace drake {
namespace examples {
namespace manipulation_station {

using Eigen::Isometry3d;
using Eigen::Vector3d;
using multibody::MultibodyPlant;
using multibody::Parser;
using robotlocomotion::image_array_t;

// TODO(russt): Consider taking DrakeLcmInterface as an argument instead of
// (only) constructing one internally.
ManipulationStationHardwareInterface::ManipulationStationHardwareInterface(
    std::vector<std::string> camera_names)
    : owned_controller_plant_(std::make_unique<MultibodyPlant<double>>()),
      owned_lcm_(new lcm::DrakeLcm()),
      camera_names_(std::move(camera_names)) {
  systems::DiagramBuilder<double> builder;

  // Publish IIWA command.
  auto iiwa_command_sender =
      builder.AddSystem<examples::kuka_iiwa_arm::IiwaCommandSender>();
  auto iiwa_command_publisher = builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<drake::lcmt_iiwa_command>(
          "IIWA_COMMAND", owned_lcm_.get()));
  // IIWA driver won't respond faster than 200Hz.
  iiwa_command_publisher->set_publish_period(0.005);
  builder.ExportInput(iiwa_command_sender->get_position_input_port(),
                      "iiwa_position");
  builder.ExportInput(iiwa_command_sender->get_torque_input_port(),
                      "iiwa_feedforward_torque");
  builder.Connect(iiwa_command_sender->get_output_port(0),
                  iiwa_command_publisher->get_input_port());

  // Receive IIWA status and populate the output ports.
  auto iiwa_status_receiver =
      builder.AddSystem<examples::kuka_iiwa_arm::IiwaStatusReceiver>();
  iiwa_status_subscriber_ = builder.AddSystem(
      systems::lcm::LcmSubscriberSystem::Make<drake::lcmt_iiwa_status>(
          "IIWA_STATUS", owned_lcm_.get()));

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
                  iiwa_status_receiver->get_input_port(0));

  // Publish WSG command.
  auto wsg_command_sender =
      builder.AddSystem<manipulation::schunk_wsg::SchunkWsgCommandSender>();
  auto wsg_command_publisher = builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<drake::lcmt_schunk_wsg_command>(
          "SCHUNK_WSG_COMMAND", owned_lcm_.get()));
  // Schunk driver won't respond faster than 20Hz.
  wsg_command_publisher->set_publish_period(0.05);
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
          "SCHUNK_WSG_STATUS", owned_lcm_.get()));
  builder.ExportOutput(wsg_status_receiver->get_state_output_port(),
                       "wsg_state_measured");
  builder.ExportOutput(wsg_status_receiver->get_force_output_port(),
                       "wsg_force_measured");
  builder.Connect(wsg_status_subscriber_->get_output_port(),
                  wsg_status_receiver->get_input_port(0));

  for (const std::string& name : camera_names_) {
    auto camera_subscriber = builder.AddSystem(
        systems::lcm::LcmSubscriberSystem::Make<image_array_t>(
            "DRAKE_RGBD_CAMERA_IMAGES_" + name, owned_lcm_.get()));
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
  const auto controller_iiwa_model =
      parser.AddModelFromFile(iiwa_sdf_path, "iiwa");
  // TODO(russt): Provide API for changing the base coordinates of the plant.
  owned_controller_plant_->WeldFrames(owned_controller_plant_->world_frame(),
                                      owned_controller_plant_->GetFrameByName(
                                          "iiwa_link_0", controller_iiwa_model),
                                      Isometry3d::Identity());
  owned_controller_plant_
      ->template AddForceElement<multibody::UniformGravityFieldElement>();
  owned_controller_plant_->Finalize();
}

void ManipulationStationHardwareInterface::Connect(bool wait_for_cameras) {
  owned_lcm_->StartReceiveThread();

  std::cout << "Waiting for IIWA status message..." << std::flush;
  iiwa_status_subscriber_->WaitForMessage(0);
  std::cout << "Received!" << std::endl;

  std::cout << "Waiting for WSG status message..." << std::flush;
  wsg_status_subscriber_->WaitForMessage(0);
  std::cout << "Received!" << std::endl;

  if (wait_for_cameras) {
    std::cout << "Waiting for cameras..." << std::flush;
    for (const auto* sub : camera_subscribers_) {
      sub->WaitForMessage(0);
    }
    std::cout << "Received!" << std::endl;
  }
}

}  // namespace manipulation_station
}  // namespace examples
}  // namespace drake
