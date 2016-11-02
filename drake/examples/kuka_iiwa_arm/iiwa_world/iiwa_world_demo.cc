#include <iostream>
#include <memory>

#include <gflags/gflags.h>

//#include "drake/common/text_logging_gflags.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_world/iiwa_world_simulator.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

namespace {

int main(int argc, char *argv[]) {
//  gflags::ParseCommandLineFlags(&argc, &argv, true);
//  logging::HandleSpdlogGflags();
//
//  // TODO(jwnimmer-tri) Allow for multiple simple cars.
//  if (FLAGS_num_simple_car > 1) {
//    std::cerr << "ERROR: Only one simple car is supported (for now)."
//              << std::endl;
//    return 1;
//  }

  auto simulator = std::make_unique < IiwaWorldSimulator < double >> ();
  simulator->AddIiwaArm();

  //simulator->AddObject(table);

//  for (int i = 0; i < FLAGS_num_simple_car; ++i) {
//    simulator->AddSimpleCar();
//  }
//  for (int i = 0; i < FLAGS_num_trajectory_car; ++i) {
//    const auto &params = CreateTrajectoryParams(i);
//    simulator->AddTrajectoryCar(
//        std::get<0>(params),
//        std::get<1>(params),
//        std::get<2>(params));
//  }

  simulator->Build();

  while (true) {
    simulator->StepBy(0.01);
  }

  return 0;
}

}  // namespace
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  return drake::examples::kuka_iiwa_arm::main(argc, argv);
}
