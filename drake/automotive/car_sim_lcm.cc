#include "drake/automotive/car_simulation.h"
#include "drake/system1/LCMSystem.h"
#include "drake/system1/LinearSystem.h"
#include "drake/system1/pd_control_system.h"
#include "drake/systems/plants/BotVisualizer.h"
#include "drake/systems/plants/parser_model_instance_id_table.h"
#include "drake/systems/plants/RigidBodySystem.h"
#include "drake/util/drakeAppUtil.h"
#include "drake/lcmt_driving_command_t.hpp"

using Eigen::VectorXd;

namespace drake {
namespace automotive {
namespace {

int main(int argc, const char* argv[]) {
  // Initializes the communication layer.
  std::shared_ptr<lcm::LCM> lcm = std::make_shared<lcm::LCM>();

  // Instantiates a duration variable that will be set by the call to
  // CreateRigidBodySystem() below.
  double duration = std::numeric_limits<double>::infinity();

  drake::parsers::ModelInstanceIdTable model_instances;

  // Initializes the rigid body system.
  auto rigid_body_sys = CreateRigidBodySystem(argc, argv, &duration,
      &model_instances);

  // Initializes and cascades all of the other systems.
  auto vehicle_sys = CreateVehicleSystem(rigid_body_sys);

  auto const& tree = rigid_body_sys->getRigidBodyTree();
  auto visualizer =
      std::make_shared<BotVisualizer<RigidBodySystem::StateVector>>(lcm, tree);

  auto sys = cascade(vehicle_sys, visualizer);

  // Initializes the simulation options.
  SimulationOptions options =
      GetCarSimulationDefaultOptions();

  // Defines the start time of the simulation.
  const double kStartTime = 0;

  // Starts the simulation.
  drake::runLCM(sys, lcm, kStartTime, duration,
                GetInitialState(*(rigid_body_sys.get())),
                options);

  return 0;
}

}  // namespace
}  // namespace automotive
}  // namespace drake

int main(int argc, const char* argv[]) {
  return drake::automotive::main(argc, argv);
}
