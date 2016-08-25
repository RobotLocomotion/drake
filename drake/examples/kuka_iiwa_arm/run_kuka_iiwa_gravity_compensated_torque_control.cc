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

// TODO(naveenoid) : Combine common components with
// run_kuka_iiwa_gravity_compensated_position_control into a class
// with a common method.

int DoMain(int argc, char* argv[]) {
  std::shared_ptr<RigidBodySystem> iiwa_system = CreateKukaIiwaSystem();

  double kDuration = 0.75;
  double kInputTorqueMagnitude = 1.75;

  // Searches through the command line looking for a "--duration" flag followed
  // by a floating point number that specifies a custom duration, and a
  // "--magnitude" flag followed by a floating point number that specifies the
  // custom magnitude of the excitation applied to the 5th joint along the
  // chain.
  for (int i = 1; i < argc; ++i) {
    if (std::string(argv[i]) == "--duration") {
      if (++i == argc) {
        throw std::runtime_error(
            "ERROR: Command line option \"--duration\" is not followed by a "
            "value!");
      }
      kDuration = atof(argv[i]);
    }

    if (std::string(argv[i]) == "--magnitude") {
      if (++i == argc) {
        throw std::runtime_error(
            "ERROR: Command line option \"--magnitude\" is not followed by a "
            "value!");
      }
      kInputTorqueMagnitude = atof(argv[i]);
    }
  }

  const int kNumDof = 7;

  // Applies a small input torque on 5th joint.
  VectorXd input_torque_vector = VectorXd::Zero(kNumDof);
  input_torque_vector(4) = kInputTorqueMagnitude;

  // The input torque is generated from a constant output AffineSystem.
  // The individual matrices of the AffineSystem are all set to zero barring
  // the initial output y0 which is then set to the dimension of the inputs
  // to the IIWA System. For more details please see :
  // http://drake.mit.edu/doxygen_cxx/classdrake_1_1_affine_system.html
  auto input_torque = std::make_shared<
      AffineSystem<NullVector, NullVector, RigidBodySystem::StateVector>>(
      MatrixXd::Zero(0, 0), MatrixXd::Zero(0, 0), VectorXd::Zero(0),
      MatrixXd::Zero(kNumDof, 0), MatrixXd::Zero(kNumDof, 0),
      input_torque_vector);

  auto lcm = std::make_shared<lcm::LCM>();
  auto visualizer = CreateKukaIiwaVisualizer(iiwa_system, lcm);

  auto controlled_robot =
      std::allocate_shared<GravityCompensatedSystem<RigidBodySystem>>(
          Eigen::aligned_allocator<GravityCompensatedSystem<RigidBodySystem>>(),
          iiwa_system);

  auto sys = cascade(cascade(input_torque, controlled_robot), visualizer);

  // Obtains an initial state of the simulation.
  VectorXd x0 = GenerateArbitraryIiwaInitialState();
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
