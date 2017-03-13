#include <limits>
#include <sstream>
#include <string>

#include <gflags/gflags.h>

#include "drake/automotive/automotive_simulator.h"
#include "drake/automotive/create_trajectory_params.h"
#include "drake/automotive/maliput/dragway/road_geometry.h"
#include "drake/common/drake_path.h"
#include "drake/common/text_logging_gflags.h"

DEFINE_string(simple_car_names, "",
              "A comma-separated list (e.g. 'Russ,Jeremy,Liang' would spawn 3 "
              "cars subscribed to DRIVING_COMMAND_Russ, "
              "DRIVING_COMMAND_Jeremy, and DRIVING_COMMAND_Liang)");
DEFINE_int32(num_trajectory_car, 1, "Number of TrajectoryCar vehicles");
DEFINE_double(target_realtime_rate, 1.0,
              "Playback speed.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");
DEFINE_double(simulation_sec, std::numeric_limits<double>::infinity(),
              "Number of seconds to simulate.");

DEFINE_int32(num_dragway_lanes, 0,
             "The number of lanes on the dragway. The number of lanes is by "
             "default zero to disable the dragway. A dragway road network is "
             "only enabled when the user specifies a number of lanes greater "
             "than zero.");
DEFINE_double(dragway_length, 100, "The length of the dragway.");
DEFINE_double(dragway_lane_width, 3.7, "The dragway lane width.");
DEFINE_double(dragway_shoulder_width, 3.0, "The dragway's shoulder width.");
DEFINE_double(dragway_base_speed, 4.0,
              "The speed of the vehicles on the right-most lane of the "
              "dragway.");
DEFINE_double(dragway_lane_speed_delta, 2,
              "The change in vehicle speed in the left-adjacent lane. For "
              "example, suppose the dragway has 3 lanes. Vehicles in the "
              "right-most lane will travel at dragway_base_speed m/s. "
              "Vehicles in the middle lane will travel at "
              "dragway_base_speed + dragway_lane_speed_delta m/s. Finally, "
              "vehicles in the left-most lane will travel at "
              "dragway_base_speed + 2 * dragway_lane_speed_delta m/s.");
DEFINE_double(dragway_vehicle_delay, 3,
              "The starting time delay between consecutive vehicles on a "
              "lane.");

namespace drake {
namespace automotive {
namespace {

enum class RoadNetworkType {
  flat = 0,
  dragway = 1
};

std::string MakeChannelName(const std::string& name) {
  const std::string default_prefix{"DRIVING_COMMAND"};
  if (name.empty()) {
    return default_prefix;
  }
  return default_prefix + "_" + name;
}

// Initializes the provided `simulator` with user-specified numbers of
// `SimpleCar` vehicles and `TrajectoryCar` vehicles. If parameter
// `road_network_type` equals `RoadNetworkType::dragway`, the provided
// `road_geometry` parameter must not be `nullptr`.
void AddVehicles(RoadNetworkType road_network_type,
    const maliput::api::RoadGeometry* road_geometry,
    AutomotiveSimulator<double>* simulator) {
  // TODO(liang.fok): Generalize this demo to allow arbitrary models to be
  // specified via command line parameters. This will involve removing some
  // hard-coded assumptions about the model's geometry. For exeample, the call
  // to CreateTrajectoryParams() below expects a "car" to have a particular
  // length and width.
  const std::string kSdfFile =
      GetDrakePath() + "/automotive/models/prius/prius_with_lidar.sdf";

  if (FLAGS_simple_car_names.empty()) {
    const std::string name = "";
    const std::string& channel_name = MakeChannelName(name);
    drake::log()->info("Adding simple car subscribed to {}.", channel_name);
    simulator->AddSimpleCarFromSdf(kSdfFile, name, channel_name);
  } else {
    std::istringstream simple_car_name_stream(FLAGS_simple_car_names);
    std::string name;
    while (getline(simple_car_name_stream, name, ',')) {
      const std::string& channel_name = MakeChannelName(name);
      drake::log()->info("Adding simple car subscribed to {}.", channel_name);
      simulator->AddSimpleCarFromSdf(kSdfFile, name, channel_name);
    }
  }

  if (road_network_type == RoadNetworkType::dragway) {
    DRAKE_DEMAND(road_geometry != nullptr);
    const maliput::dragway::RoadGeometry* dragway_road_geometry =
        dynamic_cast<const maliput::dragway::RoadGeometry*>(road_geometry);
    DRAKE_DEMAND(dragway_road_geometry != nullptr);
    for (int i = 0; i < FLAGS_num_trajectory_car; ++i) {
      const int lane_index = i % FLAGS_num_dragway_lanes;
      const double speed = FLAGS_dragway_base_speed +
          lane_index * FLAGS_dragway_lane_speed_delta;
      const double start_time = i / FLAGS_num_dragway_lanes *
           FLAGS_dragway_vehicle_delay;
      const auto& params = CreateTrajectoryParamsForDragway(
          *dragway_road_geometry, lane_index, speed, start_time);
      simulator->AddTrajectoryCarFromSdf(kSdfFile,
                                         std::get<0>(params),
                                         std::get<1>(params),
                                         std::get<2>(params));
    }
  } else {
    for (int i = 0; i < FLAGS_num_trajectory_car; ++i) {
      const auto& params = CreateTrajectoryParams(i);
      simulator->AddTrajectoryCarFromSdf(kSdfFile,
                                         std::get<0>(params),
                                         std::get<1>(params),
                                         std::get<2>(params));
    }
  }
}

// Adds a flat terrain to the provided `simulator`.
void AddFlatTerrain(AutomotiveSimulator<double>* simulator) {
  // Intentially do nothing. This is possible since only non-physics-based
  // vehicles are supported and they will not fall through the "ground" when no
  // flat terrain is present.
  //
  // TODO(liang.fok): Once physics-based vehicles are supported, a flat terrain
  // should actually be added. This can be done by adding method
  // `AutomotiveSimulator::AddFlatTerrain()`, which can then call
  // `drake::multibody::AddFlatTerrainToWorld()`. This method is defined in
  // drake/multibody/rigid_body_tree_construction.h.
}

// Adds a dragway to the provided `simulator`. The number of lanes, lane width,
// lane length, and the shoulder width are all user-specifiable via command line
// flags.
const maliput::api::RoadGeometry* AddDragway(
    AutomotiveSimulator<double>* simulator) {
  std::unique_ptr<const maliput::api::RoadGeometry> road_geometry
      = std::make_unique<const maliput::dragway::RoadGeometry>(
          maliput::api::RoadGeometryId({"Automotive Demo Dragway"}),
          FLAGS_num_dragway_lanes,
          FLAGS_dragway_length,
          FLAGS_dragway_lane_width,
          FLAGS_dragway_shoulder_width);
  return simulator->SetRoadGeometry(std::move(road_geometry));
}

// Adds a terrain to the simulated world. The type of terrain added depends on
// the provided `road_network_type` parameter. A pointer to the road network is
// returned. A return value of `nullptr` is possible if no road network is
// added.
const maliput::api::RoadGeometry* AddTerrain(RoadNetworkType road_network_type,
    AutomotiveSimulator<double>* simulator) {
  const maliput::api::RoadGeometry* road_geometry{nullptr};
  switch (road_network_type) {
    case RoadNetworkType::flat: {
      AddFlatTerrain(simulator);
      break;
    }
    case RoadNetworkType::dragway: {
      road_geometry = AddDragway(simulator);
      break;
    }
  }
  return road_geometry;
}

// Determines and returns the road network type based on the command line
// arguments.
RoadNetworkType DetermineRoadNetworkType() {
  if (FLAGS_num_dragway_lanes > 0) {
    return RoadNetworkType::dragway;
  } else {
    return RoadNetworkType::flat;
  }
}

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  logging::HandleSpdlogGflags();
  const RoadNetworkType road_network_type = DetermineRoadNetworkType();
  auto simulator = std::make_unique<AutomotiveSimulator<double>>();
  const maliput::api::RoadGeometry* road_geometry =
      AddTerrain(road_network_type, simulator.get());
  AddVehicles(road_network_type, road_geometry, simulator.get());
  simulator->Start(FLAGS_target_realtime_rate);
  simulator->StepBy(FLAGS_simulation_sec);
  return 0;
}

}  // namespace
}  // namespace automotive
}  // namespace drake

int main(int argc, char* argv[]) { return drake::automotive::main(argc, argv); }
