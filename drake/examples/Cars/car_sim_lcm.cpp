#include "drake/examples/Cars/car.h"
#include "drake/examples/Cars/gen/driving_command.h"
#include "drake/systems/LCMSystem.h"
#include "drake/systems/LinearSystem.h"
#include "drake/systems/pd_control_system.h"
#include "drake/systems/plants/BotVisualizer.h"
#include "drake/systems/plants/RigidBodySystem.h"
#include "drake/util/drakeAppUtil.h"
#include "lcmtypes/drake/lcmt_driving_command_t.hpp"
#include "drake/examples/Cars/gen/driving_command.h"

using Drake::BotVisualizer;
using Drake::DrivingCommand;
using Drake::Gain;
using Drake::SimulationOptions;

using Eigen::VectorXd;

/** Driving Simulator
 * Usage:  simulateLCM vehicle_model_file [world_model files ...]
 */

int main(int argc, char* argv[]) {
  // Initializes communication layer(s).
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
  drake::SetSimulationOptions(&options);

  // Starts the simulation.
  Drake::runLCM(sys, lcm, 0, std::numeric_limits<double>::infinity(), drake::GetInitialState(rigid_body_sys), options);
  return 0;
}
