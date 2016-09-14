#include "drake/automotive/car_simulation.h"
#include "drake/systems/LCMSystem.h"
#include "drake/systems/LinearSystem.h"
#include "drake/systems/pd_control_system.h"
#include "drake/systems/plants/BotVisualizer.h"
#include "drake/systems/plants/parser_model_instance_id_table.h"
#include "drake/systems/plants/RigidBodySystem.h"
#include "drake/util/drakeAppUtil.h"
#include "lcmtypes/drake/lcmt_driving_command_t.hpp"

using Eigen::VectorXd;

namespace drake {
namespace cars {
namespace {

int do_main(int argc, const char* argv[]) {
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
}  // namespace cars
}  // namespace drake

int main(int argc, const char* argv[]) {
  return drake::cars::do_main(argc, argv);
}
