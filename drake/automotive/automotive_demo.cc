#include <iostream>
#include <memory>
#include <sstream>
#include <string>

#include <gflags/gflags.h>

#include "drake/automotive/automotive_simulator.h"
#include "drake/automotive/create_trajectory_params.h"
#include "drake/common/drake_path.h"
#include "drake/common/text_logging_gflags.h"

DEFINE_string(simple_car_channel_postfix, "",
              "Add a simple car subscribed to each channel in a "
              "comma-separated list (e.g. 'Russ,Jeremy,Liang' would spawn 3 "
              "cars subscribed to DRIVING_COMMAND_Russ, "
              "DRIVING_COMMAND_Jeremy, and DRIVING_COMMAND_Liang)");
DEFINE_int32(num_trajectory_car, 1, "Number of TrajectoryCar vehicles");

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

  if (FLAGS_simple_car_channel_postfix.empty()) {
    std::cout << "Adding simple car subscribed to DRIVING_COMMAND" << std::endl;
    simulator->AddSimpleCarFromSdf(kSdfFile);
  } else {
    std::istringstream f(FLAGS_simple_car_channel_postfix);
    std::string s;
    while (getline(f, s, ',')) {
      if (s.empty()) {
        std::cout << "Adding simple car subscribed to DRIVING_COMMAND" << std::endl;
      } else {
        std::cout << "Adding simple car subscribed to DRIVING_COMMAND_" << s << std::endl;
      }
      simulator->AddSimpleCarFromSdf(kSdfFile,s);
    }
  }

  for (int i = 0; i < FLAGS_num_trajectory_car; ++i) {
    const auto& params = CreateTrajectoryParams(i);
    simulator->AddTrajectoryCarFromSdf(kSdfFile, std::get<0>(params),
                                       std::get<1>(params),
                                       std::get<2>(params));
  }

  simulator->Start();

  while (true) {
    simulator->StepBy(0.01);
  }

  return 0;
}

}  // namespace
}  // namespace automotive
}  // namespace drake

int main(int argc, char* argv[]) { return drake::automotive::main(argc, argv); }
