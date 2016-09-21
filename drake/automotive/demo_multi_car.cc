#include <memory>

#include "drake/automotive/create_trajectory_params.h"
#include "drake/automotive/automotive_simulator.h"

namespace drake {
namespace automotive {
namespace {

int do_main(int argc, const char* argv[]) {
  int num_cars = 100;
  if (argc == 2) {
    num_cars = atoi(argv[1]);
    if (num_cars < 1) {
      std::cerr << "The number of cars must be >= 1.\n";
      std::exit(1);
    }
  }

  auto simulator = std::make_unique<AutomotiveSimulator<double>>();

  // Add all of the desired cars.
  for (int i = 0; i < num_cars; ++i) {
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

int main(int argc, const char* argv[]) {
  return drake::automotive::do_main(argc, argv);
}
