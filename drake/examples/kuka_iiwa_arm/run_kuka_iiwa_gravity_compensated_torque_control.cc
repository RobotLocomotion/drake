#include "drake/common/drake_path.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_simulation.h"
#include "drake/systems/LCMSystem.h"
#include "drake/systems/LinearSystem.h"
#include "drake/systems/Simulation.h"
#include "drake/systems/cascade_system.h"
#include "drake/systems/gravity_compensated_system.h"
#include "drake/systems/plants/BotVisualizer.h"

using drake::AffineSystem;
using drake::BotVisualizer;
using drake::GravityCompensatedSystem;
using drake::RigidBodySystem;
using drake::SimulationOptions;
using Eigen::VectorXd;
using Eigen::MatrixXd;

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {

int DoMain(int argc, char* argv[]) {
  std::shared_ptr<RigidBodySystem> iiwa_system = CreateKukaIiwaSystem();

  double kDuration = 0.75;
  double kInputTorqueMagnitude = 1.75;

  // Searches through the command line looking for a "--duration" flag followed
  // by a floating point number that specifies a custom duration.
  for (int ii = 1; ii < argc; ++ii) {
    if (std::string(argv[ii]) == "--duration") {
      if (++ii == argc) {
        throw std::runtime_error(
            "ERROR: Command line option \"--duration\" is not followed by a "
            "value!");
      }
      kDuration = atof(argv[ii]);
    }

    if (std::string(argv[ii]) == "--magnitude") {
      if (++ii == argc) {
        throw std::runtime_error(
            "ERROR: Command line option \"--magnitude\" is not followed by a "
            "value!");
      }
      kInputTorqueMagnitude = atof(argv[ii]);
    }
  }

  const int num_dof = 7;

  // Applying a small input torque on 5th joint.
  VectorXd input_torque_vector = VectorXd::Zero(num_dof);
  input_torque_vector(4) = kInputTorqueMagnitude;
  auto input_torque = std::make_shared<
      AffineSystem<NullVector, NullVector, RigidBodySystem::StateVector>>(
      MatrixXd::Zero(0, 0), MatrixXd::Zero(0, 0), VectorXd::Zero(0),
      MatrixXd::Zero(num_dof, 0), MatrixXd::Zero(num_dof, 0),
      input_torque_vector);

  auto visualizer = CreateKukaIiwaVisualizer(iiwa_system);

  auto controlled_robot =
      std::allocate_shared<GravityCompensatedSystem<RigidBodySystem>>(
          Eigen::aligned_allocator<GravityCompensatedSystem<RigidBodySystem>>(),
          iiwa_system);

  auto sys = cascade(cascade(input_torque, controlled_robot), visualizer);

  // Obtains an initial state of the simulation.
  VectorXd x0 = ArbitraryIiwaInitialState();
  // Specifies the start time of the simulation.
  const double kStartTime = 0;

  SimulationOptions options = SetupSimulation();

  simulate(*sys.get(), kStartTime, kDuration, x0, options);

  return 0;
}

}  // namespace
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  return drake::examples::kuka_iiwa_arm::DoMain(argc, argv);
}
