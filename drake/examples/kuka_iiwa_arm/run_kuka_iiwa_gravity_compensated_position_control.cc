#include <gflags/gflags.h>

#include "drake/common/drake_path.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_simulation.h"
#include "drake/system1/LCMSystem.h"
#include "drake/system1/LinearSystem.h"
#include "drake/system1/cascade_system.h"
#include "drake/system1/gravity_compensated_pd_position_control_system.h"
#include "drake/systems/plants/BotVisualizer.h"

using drake::AffineSystem;
using drake::BotVisualizer;
using drake::CascadeSystem;
using drake::GravityCompensatedPDPositionControlSystem;
using drake::RigidBodySystem;
using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {

DEFINE_double(duration, 0.75, "Simulation duration");

// TODO(naveenoid) : Combine common code with
// run_kuka_iiwa_gravity_compensated_torque_control into a class
// with a common method.

int main(int argc, char* argv[]) {
  std::shared_ptr<RigidBodySystem> iiwa_system = CreateKukaIiwaSystem();

  double kDuration = 0.75;

  gflags::ParseCommandLineFlags(&argc, &argv, true);
  logging::HandleSpdlogGflags();
  kDuration = FLAGS_duration;

  const int kNumDof = 7;

  // Smaller gains intentionally used for demo. Gravity compensation enables
  // the usage of small feedback gains and this example demonstrates that.
  const double Kp_common = 10.0;  // Units : Nm/rad
  const double Kd_common = 0.30;  // Units : Nm/rad/sec
  VectorXd Kpdiag = VectorXd::Constant(kNumDof, Kp_common);
  VectorXd Kddiag = VectorXd::Constant(kNumDof, Kd_common);

  MatrixXd Kp = Kpdiag.asDiagonal();
  MatrixXd Kd = Kddiag.asDiagonal();

  // Obtains an initial state of the simulation.
  VectorXd x0 = GenerateArbitraryIiwaInitialState();

  // The setpoint is generated from a constant output AffineSystem.
  // The individual matrices of the AffineSystem are all set to zero barring
  // the initial output y0 which is then of the same dimension as the DoF
  // in the IIWA System. For more details please see :
  // http://drake.mit.edu/doxygen_cxx/classdrake_1_1_affine_system.html
  VectorXd set_point_vector = x0.head(kNumDof);
  auto set_point = std::make_shared<
      AffineSystem<NullVector, NullVector, RigidBodySystem::StateVector>>(
      MatrixXd::Zero(0, 0), MatrixXd::Zero(0, 0), VectorXd::Zero(0),
      MatrixXd::Zero(kNumDof, 0), MatrixXd::Zero(kNumDof, 0), set_point_vector);

  auto controlled_robot = std::allocate_shared<
      GravityCompensatedPDPositionControlSystem<RigidBodySystem>>(
      Eigen::aligned_allocator<
          GravityCompensatedPDPositionControlSystem<RigidBodySystem>>(),
      iiwa_system, Kp, Kd);

  auto lcm = std::make_shared<lcm::LCM>();
  auto visualizer = CreateKukaIiwaVisualizer(iiwa_system, lcm);

  auto sys = cascade(cascade(set_point, controlled_robot), visualizer);

  // Specifies the start time of the simulation.
  const double kStartTime = 0;

  drake::SimulationOptions options = SetupSimulation();

  drake::simulate(*sys.get(), kStartTime, kDuration, x0, options);

  return 0;
}

}  // namespace
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  return drake::examples::kuka_iiwa_arm::main(argc, argv);
}
