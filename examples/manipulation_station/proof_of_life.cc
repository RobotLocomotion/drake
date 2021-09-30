#include <gflags/gflags.h>

#include "drake/common/eigen_types.h"
#include "drake/common/find_resource.h"
#include "drake/common/is_approx_equal_abstol.h"
#include "drake/examples/manipulation_station/manipulation_station.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/contact_results_to_lcm.h"
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
using math::RigidTransform;
using math::RollPitchYaw;
using math::RotationMatrix;

DEFINE_double(target_realtime_rate, 1.0,
              "Playback speed.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");
DEFINE_double(duration, 4.0, "Simulation duration.");
DEFINE_bool(test, false, "Disable random initial conditions in test mode.");
DEFINE_string(setup, "clutter_clearing", "Manipulation Station setup option.");

int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  systems::DiagramBuilder<double> builder;

  // Create the "manipulation station".
  auto station = builder.AddSystem<ManipulationStation>();
  if (FLAGS_setup == "clutter_clearing") {
    station->SetupClutterClearingStation(std::nullopt);
    station->AddManipulandFromFile(
        "drake/manipulation/models/ycb/sdf/003_cracker_box.sdf",
        RigidTransform<double>(RollPitchYaw<double>(-1.57, 0, 3),
                               Eigen::Vector3d(-0.3, -0.55, 0.36)));
  } else if (FLAGS_setup == "manipulation_class") {
    station->SetupManipulationClassStation();
    station->AddManipulandFromFile(
        "drake/examples/manipulation_station/models/061_foam_brick.sdf",
        RigidTransform<double>(RotationMatrix<double>::Identity(),
                               Eigen::Vector3d(0.6, 0, 0)));
  } else if (FLAGS_setup == "planar") {
    station->SetupPlanarIiwaStation();
    station->AddManipulandFromFile(
        "drake/examples/manipulation_station/models/061_foam_brick.sdf",
        RigidTransform<double>(RotationMatrix<double>::Identity(),
                               Eigen::Vector3d(0.6, 0, 0)));
  } else {
    throw std::domain_error(
        "Unrecognized setup option. Options are "
        "{manipulation_class, clutter_clearing}.");
  }
  station->Finalize();

  geometry::DrakeVisualizerd::AddToBuilder(
      &builder, station->GetOutputPort("query_object"));
  multibody::ConnectContactResultsToDrakeVisualizer(
      &builder, station->get_mutable_multibody_plant(),
      station->get_scene_graph(), station->GetOutputPort("contact_results"));

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
      systems::lcm::LcmPublisherSystem::Make<lcmt_image_array>(
          "DRAKE_RGBD_CAMERA_IMAGES", nullptr,
          1.0 / 10 /* 10 fps publish period */));
  image_array_lcm_publisher->set_name("rgbd_publisher");
  builder.Connect(image_to_lcm_image_array->image_array_t_msg_output_port(),
                  image_array_lcm_publisher->get_input_port());

  auto diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);
  auto& station_context = diagram->GetMutableSubsystemContext(
      *station, &simulator.get_mutable_context());

  // Position command should hold the arm at the initial state.
  Eigen::VectorXd q0 = station->GetIiwaPosition(station_context);
  station->GetInputPort("iiwa_position").FixValue(&station_context, q0);

  // Zero feed-forward torque.
  station->GetInputPort("iiwa_feedforward_torque")
      .FixValue(&station_context, VectorXd::Zero(station->num_iiwa_joints()));

  // Nominal WSG position is open.
  station->GetInputPort("wsg_position").FixValue(&station_context, 0.1);
  // Force limit at 40N.
  station->GetInputPort("wsg_force_limit").FixValue(&station_context, 40.0);

  if (!FLAGS_test) {
    std::random_device rd;
    RandomGenerator generator{rd()};
    diagram->SetRandomContext(&simulator.get_mutable_context(), &generator);
  }

  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.AdvanceTo(FLAGS_duration);

  // Check that the arm is (very roughly) in the commanded position.
  VectorXd q = station->GetIiwaPosition(station_context);
  if (!is_approx_equal_abstol(q, q0, 1.e-3)) {
    std::cout << "q is not sufficiently close to q0.\n";
    std::cout << "q - q0  = " << (q - q0).transpose() << "\n";
    return EXIT_FAILURE;
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
