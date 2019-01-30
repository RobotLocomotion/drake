#include <gflags/gflags.h>
#include <iostream>
#include <fstream>

#include "drake/common/eigen_types.h"
#include "drake/common/find_resource.h"
#include "drake/common/is_approx_equal_abstol.h"
#include "drake/examples/manipulation_station/manipulation_station.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/contact_results_to_lcm.h"
#include "drake/perception/depth_image_to_point_cloud.h"
#include "drake/perception/point_cloud.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/sensors/camera_info.h"
#include "drake/systems/sensors/image_to_lcm_image_array_t.h"

namespace drake {
namespace examples {
namespace manipulation_station {
namespace {

// Starts up bin-picking simulation with textured YCB Objects
// and saves the point cloud of the scene to a text file.

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
  station->SetupDefaultStation();
  station->Finalize();

  geometry::ConnectDrakeVisualizer(&builder, station->get_mutable_scene_graph(),
                                   station->GetOutputPort("pose_bundle"));
  multibody::ConnectContactResultsToDrakeVisualizer(
      &builder, station->get_mutable_multibody_plant(),
      station->GetOutputPort("contact_results"));

  auto image_to_lcm_image_array =
      builder.template AddSystem<systems::sensors::ImageToLcmImageArrayT>();
  image_to_lcm_image_array->set_name("converter");
  // TODO(kmuhlrad): build a proper map for the DUTs
  systems::sensors::CameraInfo camera_info(848, 480, 0.712);
  auto dut = builder.AddSystem<perception::DepthImageToPointCloud>(
      camera_info, systems::sensors::PixelType::kDepth16U);
  for (const auto& name : station->get_camera_names()) {
    const auto& cam_port =
        image_to_lcm_image_array
            ->DeclareImageInputPort<systems::sensors::PixelType::kRgba8U>(
                "camera_" + name);
    builder.Connect(station->GetOutputPort("camera_" + name + "_rgb_image"),
                    cam_port);
    builder.Connect(station->GetOutputPort("camera_" + name + "_rgb_image"),
                    dut->rgb_image_input_port());
    builder.Connect(station->GetOutputPort("camera_" + name + "_depth_image"),
		    dut->depth_image_input_port());
  }
  auto image_array_lcm_publisher = builder.template AddSystem(
      systems::lcm::LcmPublisherSystem::Make<robotlocomotion::image_array_t>(
          "DRAKE_RGBD_CAMERA_IMAGES", nullptr,
          1.0 / 10 /* 10 fps publish period */));
  image_array_lcm_publisher->set_name("rgbd_publisher");
  builder.Connect(image_to_lcm_image_array->image_array_t_msg_output_port(),
                  image_array_lcm_publisher->get_input_port());

  auto diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);
  auto& station_context = diagram->GetMutableSubsystemContext(
      *station, &simulator.get_mutable_context());
  const auto& dut_context = diagram->GetMutableSubsystemContext(
      *dut, &simulator.get_mutable_context());

  // Position command should hold the arm at the initial state.
  Eigen::VectorXd q0 = station->GetIiwaPosition(station_context);
  station_context.FixInputPort(
      station->GetInputPort("iiwa_position").get_index(), q0);

  // Zero feed-forward torque.
  station_context.FixInputPort(
      station->GetInputPort("iiwa_feedforward_torque").get_index(),
      VectorXd::Zero(station->num_iiwa_joints()));

  // Nominal WSG position is open.
  station_context.FixInputPort(
      station->GetInputPort("wsg_position").get_index(), Vector1d(0.1));
  // Force limit at 40N.
  station_context.FixInputPort(
      station->GetInputPort("wsg_force_limit").get_index(), Vector1d(40.0));

  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.StepTo(FLAGS_duration);

  // Check that the arm is (very roughly) in the commanded position.
  VectorXd q = station->GetIiwaPosition(station_context);
  if (!is_approx_equal_abstol(q, q0, 1.e-3)) {
    std::cout << "q - q0  = " << (q - q0).transpose() << std::endl;
    DRAKE_ABORT_MSG("q is not sufficiently close to q0.");
  }

  auto point_cloud = dut->point_cloud_output_port()
      .Eval<perception::PointCloud>(dut_context);

  std::cout << "Point cloud size: " << point_cloud.size() << std::endl;
  std::cout << "Has xys: " << point_cloud.has_xyzs() << std::endl;
  std::cout << "Has rgbs: " << point_cloud.has_rgbs() << std::endl;
  std::cout << "has normals: " << point_cloud.has_normals() << std::endl;
  std::cout << "Point cloud channels: " << point_cloud.fields().base_fields() << std::endl;

  std::ofstream points_file;
  points_file.open("camera_points.txt");
  for (int i = 0; i < point_cloud.size(); i++) {
    auto point = point_cloud.xyz(i);
    points_file << point[0] << " " << point[1] << " " << point[2] << "\n";
  }
  points_file.close();

  std::ofstream colors_file;
  colors_file.open("camera_colors.txt");
  for (int i = 0; i < point_cloud.size(); i++) {
    auto color = point_cloud.rgb(i);
    colors_file << int(color[0]) << " " << int(color[1]) << " " << int(color[2]) << "\n";
  }
  colors_file.close();

  return 0;
}

}  // namespace
}  // namespace manipulation_station
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  return drake::examples::manipulation_station::do_main(argc, argv);
}
