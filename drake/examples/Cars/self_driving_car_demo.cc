#include "drake/examples/Cars/self_driving_car.h"
#include "drake/systems/LCMSystem.h"
#include "drake/systems/plants/BotVisualizer.h"

using Drake::BotVisualizer;
using Drake::SimulationOptions;

using Eigen::VectorXd;

namespace drake {
namespace examples {
namespace cars {
namespace {

int do_main(int argc, const char* argv[]) {
  // Initializes the communication layer.
  std::shared_ptr<lcm::LCM> lcm = std::make_shared<lcm::LCM>();

  // Instantiates a duration variable that will be set by the call to
  // CreateRigidBodySystem() below.
  double duration = std::numeric_limits<double>::infinity();

  // Instantiate car
  auto car = std::make_shared<SelfDrivingCar>(argc,argv);

  auto const& tree = car->getRigidBodyTree();

  auto visualizer =
      std::make_shared<BotVisualizer<SelfDrivingCar::StateVector>>(lcm, tree);

  auto sys = cascade(car, visualizer);

  // Initializes the simulation options.
  SimulationOptions options = Drake::default_simulation_options;
  options.initial_step_size = 5e-3;
  options.timeout_seconds = std::numeric_limits<double>::infinity();

  // Starts the simulation.
  Drake::runLCM(sys, lcm, 0, duration,
                car->ComputeInitialState(),
                options);

  return 0;
}

}  // namespace
}  // namespace cars
}  // namespace examples
}  // namespace drake

int main(int argc, const char* argv[]) {
  return drake::examples::cars::do_main(argc, argv);
}
