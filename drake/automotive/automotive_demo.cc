#include <iostream>
#include <memory>

#include <gflags/gflags.h>

#include "drake/common/text_logging_gflags.h"
#include "drake/automotive/automotive_simulator.h"
#include "drake/automotive/create_trajectory_params.h"

DEFINE_int32(num_simple_car, 1, "Number of SimpleCar vehicles");
DEFINE_int32(num_trajectory_car, 1, "Number of TrajectoryCar vehicles");

namespace drake {
namespace automotive {
namespace {

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  logging::HandleSpdlogGflags();

  // TODO(jwnimmer-tri) Allow for multiple simple cars.
  if (FLAGS_num_simple_car > 1) {
    std::cerr << "ERROR: Only one simple car is supported (for now)."
              << std::endl;
    return 1;
  }

  auto simulator = std::make_unique<AutomotiveSimulator<double>>();
  for (int i = 0; i < FLAGS_num_simple_car; ++i) {
    simulator->AddSimpleCar();
  }
  for (int i = 0; i < FLAGS_num_trajectory_car; ++i) {
    const auto& params = CreateTrajectoryParams(i);
    simulator->AddTrajectoryCar(
        std::get<0>(params),
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

int main(int argc, char* argv[]) {
  return drake::automotive::main(argc, argv);
}
