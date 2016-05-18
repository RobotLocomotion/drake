#include "drake/examples/Cars/car_simulation.h"
#include "drake/examples/Cars/gen/driving_command.h"
#include "drake/systems/LCMSystem.h"
#include "drake/systems/LinearSystem.h"
#include "drake/systems/pd_control_system.h"
#include "drake/systems/plants/BotVisualizer.h"
#include "drake/systems/plants/RigidBodySystem.h"
#include "drake/util/drakeAppUtil.h"
#include "lcmtypes/drake/lcmt_driving_command_t.hpp"

using Drake::BotVisualizer;
using Drake::Gain;
using Drake::SimulationOptions;

using Eigen::VectorXd;

namespace drake {
namespace {

int do_main(int argc, const char* argv[]) {
  // Initializes the communication layer.
  std::shared_ptr<lcm::LCM> lcm = std::make_shared<lcm::LCM>();

  // Initializes the rigid body system.
  auto rigid_body_sys = drake::CreateRigidBodySystem(argc, argv);
  auto const& tree = rigid_body_sys->getRigidBodyTree();

  // Initializes and cascades all of the other systems.
  auto vehicle_sys = drake::CreateVehicleSystem(rigid_body_sys);

  auto visualizer =
      std::make_shared<BotVisualizer<RigidBodySystem::StateVector>>(lcm, tree);

  auto sys = cascade(vehicle_sys, visualizer);

  // Initializes the simulation options.
  SimulationOptions options;
  try {
    drake::SetSimulationOptions(&options);
  } catch (std::runtime_error error) {
    std::cerr << "ERROR: Simulation options is a nullptr!" << std::endl;
    return EXIT_FAILURE;
  }

  // Obtains the desired duration of the simulation.
  double duration = std::numeric_limits<double>::infinity();
  for (int i = 2; i < argc; i++) {
    if (std::string(argv[i]) == "--duration") {
      duration = atof(argv[++i]);
    }
  }

  // Starts the simulation.
  Drake::runLCM(sys, lcm, 0, duration,
                drake::GetInitialState(*(rigid_body_sys.get())), options);

  return 0;
}

}  // namespace
}  // namespace drake

int main(int argc, const char* argv[]) {
  return drake::do_main(argc, argv);
}
