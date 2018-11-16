#include <gflags/gflags.h>

#include "drake/common/eigen_types.h"
#include "drake/common/find_resource.h"
#include "drake/common/is_approx_equal_abstol.h"
#include "drake/examples/manipulation_station/manipulation_station.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/multibody/multibody_tree/multibody_plant/contact_results_to_lcm.h"
#include "drake/multibody/multibody_tree/parsing/multibody_plant_sdf_parser.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/sensors/image_to_lcm_image_array_t.h"

namespace drake {
namespace examples {
namespace manipulation_station {
namespace {

// Simple example which simulates the manipulation station (and visualizes it
// with drake visualizer).
// TODO(russt): Replace this with a slightly more interesting minimal example
// (e.g. picking up an object) and perhaps a slightly more descriptive name.

using Eigen::VectorXd;

DEFINE_double(target_realtime_rate, 1.0,
              "Playback speed.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");
DEFINE_double(duration, 4.0, "Simulation duration.");

int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  systems::DiagramBuilder<double> builder;

  // Create the "manipulation station".
  auto station = builder.AddSystem<ManipulationStation>();
  station->AddCupboard();
  auto object = multibody::parsing::AddModelFromSdfFile(
      FindResourceOrThrow(
          "drake/examples/manipulation_station/models/061_foam_brick.sdf"),
      "object", &station->get_mutable_multibody_plant(),
      &station->get_mutable_scene_graph());
  station->Finalize();

  geometry::ConnectDrakeVisualizer(&builder, station->get_mutable_scene_graph(),
                                   station->GetOutputPort("pose_bundle"));
  multibody::multibody_plant::ConnectContactResultsToDrakeVisualizer(
      &builder, station->get_mutable_multibody_plant(),
      station->GetOutputPort("contact_results"));

  auto image_to_lcm_image_array =
      builder.template AddSystem<systems::sensors::ImageToLcmImageArrayT>();
  image_to_lcm_image_array->set_name("converter");
  for (const auto& name : station->get_camera_names()) {
    const auto& cam_port =
        image_to_lcm_image_array
            ->DeclareImageInputPort<systems::sensors::PixelType::kRgba8U>(
                "camera_" + name);
    builder.Connect(station->GetOutputPort("camera_" + name + "_rgb_image"),
                    cam_port);
  }
  auto image_array_lcm_publisher = builder.template AddSystem(
      systems::lcm::LcmPublisherSystem::Make<robotlocomotion::image_array_t>(
          "DRAKE_RGBD_CAMERA_IMAGES", nullptr));
  image_array_lcm_publisher->set_name("rgbd_publisher");
  image_array_lcm_publisher->set_publish_period(1. / 10 /* 10 fps */);
  builder.Connect(image_to_lcm_image_array->image_array_t_msg_output_port(),
                  image_array_lcm_publisher->get_input_port());

  auto diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);
  auto& station_context = diagram->GetMutableSubsystemContext(
      *station, &simulator.get_mutable_context());

  // Set initial conditions for the IIWA to a position inside the cell with
  // the hand pointed towards the work table.
  const int kNumDofIiwa = 7;
  VectorXd q0(kNumDofIiwa);
  q0 << 0, 0.6, 0, -1.75, 0, 1.0, 0;
  station->SetIiwaPosition(q0, &station_context);
  const VectorXd qdot0 = VectorXd::Zero(kNumDofIiwa);
  station->SetIiwaVelocity(qdot0, &station_context);

  // Position command should hold the arm at the initial state.
  station_context.FixInputPort(
      station->GetInputPort("iiwa_position").get_index(), q0);

  // Zero feed-forward torque.
  station_context.FixInputPort(
      station->GetInputPort("iiwa_feedforward_torque").get_index(),
      VectorXd::Zero(kNumDofIiwa));

  // Nominal WSG position is open.
  station_context.FixInputPort(
      station->GetInputPort("wsg_position").get_index(), Vector1d(0.1));
  // Force limit at 40N.
  station_context.FixInputPort(
      station->GetInputPort("wsg_force_limit").get_index(), Vector1d(40.0));

  // Place the object in the center of the table in front of the robot.
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = Eigen::Vector3d(.6, 0, 0);
  station->get_multibody_plant().tree().SetFreeBodyPoseOrThrow(
      station->get_multibody_plant().GetBodyByName("base_link", object), pose,
      &station->GetMutableSubsystemContext(station->get_multibody_plant(),
                                           &station_context));

  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.Initialize();
  simulator.StepTo(FLAGS_duration);

  // Check that the arm is (very roughly) in the commanded position.
  VectorXd q = station->GetIiwaPosition(station_context);
  if (!is_approx_equal_abstol(q, q0, 1.e-3)) {
    std::cout << "q - q0  = " << (q - q0).transpose() << std::endl;
    DRAKE_ABORT_MSG("q is not sufficiently close to q0.");
  }

  return 0;
}

}  // namespace
}  // namespace manipulation_station
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  return drake::examples::manipulation_station::do_main(argc, argv);
}
