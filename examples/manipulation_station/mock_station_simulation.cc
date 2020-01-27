#include <limits>

#include <gflags/gflags.h>

#include "drake/common/eigen_types.h"
#include "drake/common/find_resource.h"
#include "drake/common/is_approx_equal_abstol.h"
#include "drake/examples/manipulation_station/manipulation_station.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/lcmt_iiwa_command.hpp"
#include "drake/lcmt_iiwa_status.hpp"
#include "drake/lcmt_schunk_wsg_command.hpp"
#include "drake/lcmt_schunk_wsg_status.hpp"
#include "drake/manipulation/kuka_iiwa/iiwa_command_receiver.h"
#include "drake/manipulation/kuka_iiwa/iiwa_status_sender.h"
#include "drake/manipulation/schunk_wsg/schunk_wsg_lcm.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/frame_visualizer.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/primitives/matrix_gain.h"
#include "drake/systems/sensors/image_to_lcm_image_array_t.h"

#include "drake/examples/manipulation_station/pdc_ik.h"

namespace drake {
namespace examples {
namespace manipulation_station {
namespace {

// Runs a simulation of the manipulation station plant as a stand-alone
// simulation which mocks the network inputs and outputs of the real robot
// station.  This is a useful test in the transition from a single-process
// simulation to operating on the real robot hardware.

using Eigen::VectorXd;

DEFINE_double(target_realtime_rate, 1.0,
              "Playback speed.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");
DEFINE_double(duration, std::numeric_limits<double>::infinity(),
              "Simulation duration.");
DEFINE_string(setup, "pdc",
              "Manipulation station type to simulate. "
              "Can be {manipulation_class, clutter_clearing}");

int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  systems::DiagramBuilder<double> builder;

  // Create the "manipulation station".
  auto station = builder.AddSystem<ManipulationStation>();
  if (FLAGS_setup == "manipulation_class") {
    station->SetupManipulationClassStation();
    station->AddManipulandFromFile(
        "drake/examples/manipulation_station/models/061_foam_brick.sdf",
        math::RigidTransform<double>(math::RotationMatrix<double>::Identity(),
                                     Eigen::Vector3d(0.6, 0, 0)));
  } else if (FLAGS_setup == "pdc") {
    math::RigidTransform<double> X_WC(
        math::RollPitchYaw<double>(-12 * M_PI / 180., 0, M_PI_2),
        Eigen::Vector3d(0.05, 0, 0.114));
    station->SetupPdcDataCollectStation(X_WC);
    station->AddManipulandFromFile(
        "drake/manipulation/models/plastic_mug/plastic_mug.sdf",
        math::RigidTransform<double>(math::RollPitchYaw<double>(-1.57, 0, 3),
                                     Eigen::Vector3d(-0.3, -0.55, 0.36)));
  } else if (FLAGS_setup == "clutter_clearing") {
    station->SetupClutterClearingStation(std::nullopt);
    station->AddManipulandFromFile(
        "drake/manipulation/models/ycb/sdf/003_cracker_box.sdf",
        math::RigidTransform<double>(math::RollPitchYaw<double>(-1.57, 0, 3),
                                     Eigen::Vector3d(-0.3, -0.55, 0.36)));
  } else {
    throw std::domain_error(
        "Unrecognized station type. Options are "
        "{manipulation_class, clutter_clearing}.");
  }
  // TODO(russt): Load sdf objects specified at the command line.  Requires
  // #9747.
  station->Finalize();

  geometry::ConnectDrakeVisualizer(&builder, station->get_scene_graph(),
                                   station->GetOutputPort("pose_bundle"));

  auto lcm = builder.AddSystem<systems::lcm::LcmInterfaceSystem>();

  auto iiwa_command_subscriber = builder.AddSystem(
      systems::lcm::LcmSubscriberSystem::Make<drake::lcmt_iiwa_command>(
          "IIWA_COMMAND", lcm));
  auto iiwa_command =
      builder.AddSystem<manipulation::kuka_iiwa::IiwaCommandReceiver>();
  builder.Connect(iiwa_command_subscriber->get_output_port(),
                  iiwa_command->get_input_port());

  // Pull the positions out of the state.
  builder.Connect(iiwa_command->get_commanded_position_output_port(),
                  station->GetInputPort("iiwa_position"));
  builder.Connect(iiwa_command->get_commanded_torque_output_port(),
                  station->GetInputPort("iiwa_feedforward_torque"));

  auto iiwa_status =
      builder.AddSystem<manipulation::kuka_iiwa::IiwaStatusSender>();
  builder.Connect(station->GetOutputPort("iiwa_position_commanded"),
                  iiwa_status->get_position_commanded_input_port());
  builder.Connect(station->GetOutputPort("iiwa_position_measured"),
                  iiwa_status->get_position_measured_input_port());
  builder.Connect(station->GetOutputPort("iiwa_velocity_estimated"),
                  iiwa_status->get_velocity_estimated_input_port());
  builder.Connect(station->GetOutputPort("iiwa_torque_commanded"),
                  iiwa_status->get_torque_commanded_input_port());
  builder.Connect(station->GetOutputPort("iiwa_torque_measured"),
                  iiwa_status->get_torque_measured_input_port());
  builder.Connect(station->GetOutputPort("iiwa_torque_external"),
                  iiwa_status->get_torque_external_input_port());
  auto iiwa_status_publisher = builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<drake::lcmt_iiwa_status>(
          "IIWA_STATUS", lcm, 0.005 /* publish period */));
  builder.Connect(iiwa_status->get_output_port(),
                  iiwa_status_publisher->get_input_port());

  // Receive the WSG commands.
  auto wsg_command_subscriber = builder.AddSystem(
      systems::lcm::LcmSubscriberSystem::Make<drake::lcmt_schunk_wsg_command>(
          "SCHUNK_WSG_COMMAND", lcm));
  auto wsg_command =
      builder.AddSystem<manipulation::schunk_wsg::SchunkWsgCommandReceiver>();
  builder.Connect(wsg_command_subscriber->get_output_port(),
                  wsg_command->GetInputPort("command_message"));
  builder.Connect(wsg_command->get_position_output_port(),
                  station->GetInputPort("wsg_position"));
  builder.Connect(wsg_command->get_force_limit_output_port(),
                  station->GetInputPort("wsg_force_limit"));

  // Publish the WSG status.
  auto wsg_status =
      builder.AddSystem<manipulation::schunk_wsg::SchunkWsgStatusSender>();
  builder.Connect(station->GetOutputPort("wsg_state_measured"),
                  wsg_status->get_state_input_port());
  builder.Connect(station->GetOutputPort("wsg_force_measured"),
                  wsg_status->get_force_input_port());
  auto wsg_status_publisher = builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<drake::lcmt_schunk_wsg_status>(
          "SCHUNK_WSG_STATUS", lcm, 0.05 /* publish period */));
  builder.Connect(wsg_status->get_output_port(0),
                  wsg_status_publisher->get_input_port());

  // Publish the camera outputs.
  auto image_to_lcm_image_array =
      builder.template AddSystem<systems::sensors::ImageToLcmImageArrayT>();
  image_to_lcm_image_array->set_name("converter");
  for (const auto& name : station->get_camera_names()) {
    drake::log()->info("camera name: {}", name);
    const auto& rgb_port =
        image_to_lcm_image_array
            ->DeclareImageInputPort<systems::sensors::PixelType::kRgba8U>(
                "camera_" + name + "_rgb");
    builder.Connect(station->GetOutputPort("camera_" + name + "_rgb_image"),
                    rgb_port);
    const auto& depth_port =
        image_to_lcm_image_array
            ->DeclareImageInputPort<systems::sensors::PixelType::kDepth16U>(
                "camera_" + name + "_depth");
    builder.Connect(station->GetOutputPort("camera_" + name + "_depth_image"),
                    depth_port);
    const auto& label_port =
        image_to_lcm_image_array
            ->DeclareImageInputPort<systems::sensors::PixelType::kLabel16I>(
                "camera_" + name + "_label");
    builder.Connect(station->GetOutputPort("camera_" + name + "_label_image"),
                    label_port);
  }
  auto image_array_lcm_publisher = builder.template AddSystem(
      systems::lcm::LcmPublisherSystem::Make<robotlocomotion::image_array_t>(
          "DRAKE_RGBD_CAMERA_IMAGES", lcm, 1 / 30.));
  image_array_lcm_publisher->set_name("rgbd_publisher");
  builder.Connect(image_to_lcm_image_array->image_array_t_msg_output_port(),
                  image_array_lcm_publisher->get_input_port());

  // Frame Visualizer
  std::vector<const multibody::Frame<double>*> frames_to_viz;
  const auto& plant = station->get_multibody_plant();
  frames_to_viz.push_back(&plant.GetFrameByName("wrist_camera_frame"));
  auto frame_viz = builder.template AddSystem<multibody::FrameVisualizer>(
      &plant, frames_to_viz);
  auto frame_viz_lcm_publisher = builder.template AddSystem(
      systems::lcm::LcmPublisherSystem::Make<lcmt_viewer_draw>(
          "DRAKE_DRAW_FRAMES", lcm, 1 / 60.));
  frame_viz_lcm_publisher->set_name("frame_viz_lcm_publisher");
  builder.Connect(station->GetOutputPort("plant_continuous_state"),
                  frame_viz->get_input_port(0));
  builder.Connect(frame_viz->get_output_port(0),
                  frame_viz_lcm_publisher->get_input_port());

  auto diagram = builder.Build();

  // Solve
  const auto& camera_frame = plant.GetFrameByName("wrist_camera_frame");
  std::vector<Eigen::VectorXd> q_solutions = SovleGaze(
      plant, camera_frame, {Eigen::Vector3d(0.5, -0.1, 0.235)}, 15, 0.1, 1);
  drake::log()->info("q {}", q_solutions.at(0).transpose());

  systems::Simulator<double> simulator(*diagram);
  auto& context = simulator.get_mutable_context();
  auto& station_context =
      diagram->GetMutableSubsystemContext(*station, &context);

  // Get the initial Iiwa pose and initialize the iiwa_command to match.
  VectorXd q0 = station->GetIiwaPosition(station_context);
  iiwa_command->set_initial_position(
      &diagram->GetMutableSubsystemContext(*iiwa_command, &context), q0);

  simulator.set_publish_every_time_step(false);
  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.AdvanceTo(FLAGS_duration);

  return 0;
}

}  // namespace
}  // namespace manipulation_station
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  return drake::examples::manipulation_station::do_main(argc, argv);
}
