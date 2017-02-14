#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>

#include <gflags/gflags.h>

#include "drake/automotive/automotive_simulator.h"
#include "drake/automotive/create_trajectory_params.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/dragway/road_geometry.h"
#include "drake/common/drake_path.h"
#include "drake/common/text_logging_gflags.h"

using std::string;

// Shared parameters (these are parameters used by more than one demo):
DEFINE_int32(demo, 0,
    "Specifies which demo to run. Valid options include:\n"
    "  - 0: simple car and trajectory cars on a flat terrain\n"
    "  - 1: trajectory cars on multi-lane dragway");
DEFINE_double(target_realtime_rate, 1.0,
    "Playback speed.  See documentation for "
    "Simulator::set_target_realtime_rate() for details.");
DEFINE_double(simulation_sec, std::numeric_limits<double>::infinity(),
    "Number of seconds to simulate.");

// Demo 0 parameters:
DEFINE_string(simple_car_names, "",
    "A comma-separated list (e.g. 'Russ,Jeremy,Liang' would spawn 3 "
    "cars subscribed to DRIVING_COMMAND_Russ, "
    "DRIVING_COMMAND_Jeremy, and DRIVING_COMMAND_Liang)");
DEFINE_int32(num_trajectory_car, 1,
    "Number of TrajectoryCar vehicles");

// Demo 1 parameters:
DEFINE_int32(num_lanes, 1, "The number of lanes on the dragway.");
DEFINE_double(length, 100, "The length of the dragway.");
DEFINE_double(lane_width, 3.7, "The lane width.");
DEFINE_double(shoulder_width, 3.0, "The shoudler width.");

namespace drake {
namespace automotive {
namespace {

// Initializes the provided `simulator` with a user-specified number of
// SimpleCar car and TrajectoryCar cars on a flat terrain.
void InitializeDemo0(AutomotiveSimulator<double>* simulator) {
  // TODO(liang.fok): Generalize this demo to allow arbitrary models to be
  // specified via command line parameters or Protobuf configuration files.
  // This will involve removing some hard-coded assumptions about the model's
  // geometry. For exeample, the call to CreateTrajectoryParams() below
  // expects a "car" to have a particular length and width.
  const string kSdfFile =
      GetDrakePath() + "/automotive/models/prius/prius_with_lidar.sdf";

  if (FLAGS_simple_car_names.empty()) {
    std::cout << "Adding simple car subscribed to DRIVING_COMMAND."
              << std::endl;
    simulator->AddSimpleCarFromSdf(kSdfFile);
  } else {
    std::istringstream simple_car_name_stream(FLAGS_simple_car_names);
    string name;
    while (getline(simple_car_name_stream, name, ',')) {
      if (name.empty()) {
        std::cout << "Adding simple car subscribed to DRIVING_COMMAND."
                  << std::endl;
      } else {
        std::cout << "Adding simple car subscribed to DRIVING_COMMAND_" << name
                  << "." << std::endl;
      }
      simulator->AddSimpleCarFromSdf(kSdfFile, name);
    }
  }

  for (int i = 0; i < FLAGS_num_trajectory_car; ++i) {
    const auto& params = CreateTrajectoryParams(i);
    simulator->AddTrajectoryCarFromSdf(kSdfFile, std::get<0>(params),
                                       std::get<1>(params),
                                       std::get<2>(params));
  }
}

// Initializes the provided `simulator` with a dragway with one TrajectoryCar
// per lane. The number of lanes, lane width, lane length, and the shoulder
// width are all user-specifiable via command line flags.
void InitializeDemo1(AutomotiveSimulator<double>* simulator) {
  std::unique_ptr<const maliput::api::RoadGeometry> road_geometry_temp
      = std::make_unique<const maliput::dragway::RoadGeometry>(
          maliput::api::RoadGeometryId(
              {"AutomotiveDragwayWithTrajectoryCarDemo"}),
          FLAGS_num_lanes,
          FLAGS_length,
          FLAGS_lane_width,
          FLAGS_shoulder_width);
  const maliput::dragway::RoadGeometry* road_geometry =
      dynamic_cast<const maliput::dragway::RoadGeometry*>(
          simulator->SetRoadGeometry(&road_geometry_temp));
  DRAKE_DEMAND(road_geometry != nullptr);

  for (int i = 0; i < FLAGS_num_lanes; ++i) {
    const auto& params = CreateTrajectoryParamsForDragway(*road_geometry, i);
    const string kSdfFile =
        GetDrakePath() + "/automotive/models/prius/prius_with_lidar.sdf";
    simulator->AddTrajectoryCarFromSdf(kSdfFile, std::get<0>(params),
                                       std::get<1>(params),
                                       std::get<2>(params));
  }
}

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  logging::HandleSpdlogGflags();

  auto simulator = std::make_unique<AutomotiveSimulator<double>>();

  switch (FLAGS_demo) {
    case 0:
      InitializeDemo0(simulator.get());
    break;
    case 1:
      InitializeDemo1(simulator.get());
    break;
    default:
      std::cerr << "ERROR: Unsupported demo number: "
                << std::to_string(FLAGS_demo) << "." << std::endl;
      return 1;
  }

  simulator->Start(FLAGS_target_realtime_rate);
  simulator->StepBy(FLAGS_simulation_sec);
  return 0;
}

}  // namespace
}  // namespace automotive
}  // namespace drake

int main(int argc, char* argv[]) { return drake::automotive::main(argc, argv); }
