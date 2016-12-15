#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>

#include <gflags/gflags.h>

#include "drake/automotive/automotive_simulator.h"
#include "drake/automotive/create_trajectory_params.h"
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

namespace drake {
namespace automotive {
namespace {

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  logging::HandleSpdlogGflags();

  // TODO(liang.fok): Generalize this demo to allow arbitrary models to be
  // specified via command line parameters. This will involve removing some
  // hard-coded assumptions about the model's geometry. For exeample, the call
  // to CreateTrajectoryParams() below expects a "car" to have a particular
  // length and width.
  const std::string kSdfFile =
      GetDrakePath() + "/automotive/models/prius/prius_with_lidar.sdf";
  auto simulator = std::make_unique<AutomotiveSimulator<double>>();

  if (FLAGS_simple_car_names.empty()) {
    std::cout << "Adding simple car subscribed to DRIVING_COMMAND" << std::endl;
    simulator->AddSimpleCarFromSdf(kSdfFile);
  } else {
    std::istringstream simple_car_name_stream(FLAGS_simple_car_names);
    std::string name;
    while (getline(simple_car_name_stream, name, ',')) {
      if (name.empty()) {
        std::cout << "Adding simple car subscribed to DRIVING_COMMAND"
                  << std::endl;
      } else {
        std::cout << "Adding simple car subscribed to DRIVING_COMMAND_" << name
                  << std::endl;
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

  simulator->Start(FLAGS_target_realtime_rate);
  simulator->StepBy(FLAGS_simulation_sec);

  return 0;
}

}  // namespace
}  // namespace automotive
}  // namespace drake

int main(int argc, char* argv[]) { return drake::automotive::main(argc, argv); }
