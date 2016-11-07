#include <gflags/gflags.h>

#include "drake/common/drake_path.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_simulation.h"
#include "drake/system1/LCMSystem.h"
#include "drake/system1/LinearSystem.h"
#include "drake/system1/Simulation.h"
#include "drake/system1/cascade_system.h"
#include "drake/system1/gravity_compensated_system.h"
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

DEFINE_double(duration, 0.75, "Simulation duration");
DEFINE_double(magnitude, 1.75, "Joint 5 Input torque magnitude");

// TODO(naveenoid) : Combine common code with
// run_kuka_iiwa_gravity_compensated_position_control into a class
// with a common method.

int main(int argc, char* argv[]) {
  std::shared_ptr<RigidBodySystem> iiwa_system = CreateKukaIiwaSystem();

  double kDuration = 0.75;
  double kInputTorqueMagnitude = 1.75;

  gflags::ParseCommandLineFlags(&argc, &argv, true);
  logging::HandleSpdlogGflags();
  kDuration = FLAGS_duration;
  kInputTorqueMagnitude = FLAGS_magnitude;

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
  return drake::examples::kuka_iiwa_arm::main(argc, argv);
}
