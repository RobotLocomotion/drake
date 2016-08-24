#include "drake/common/drake_path.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_simulation.h"
#include "drake/systems/LCMSystem.h"
#include "drake/systems/LinearSystem.h"
#include "drake/systems/cascade_system.h"
#include "drake/systems/gravity_compensated_pd_position_control_system.h"
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

int DoMain(int argc, char* argv[]) {
  std::shared_ptr<RigidBodySystem> iiwa_system = CreateKukaIiwaSystem();

  double kDuration = 0.75;

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
  }

  const int num_dof = 7;

  // Smaller gains intentionally used for demo.
  const double Kp_common = 10.0;  // Units : Nm/rad
  const double Kd_common = 0.30;  // Units : Nm/rad/sec
  VectorXd Kpdiag = VectorXd::Constant(num_dof, Kp_common);
  VectorXd Kddiag = VectorXd::Constant(num_dof, Kd_common);

  MatrixXd Kp = Kpdiag.asDiagonal();
  MatrixXd Kd = Kddiag.asDiagonal();

  // Obtains an initial state of the simulation.
  VectorXd x0 = ArbitraryIiwaInitialState();

  // Set point is the initial configuration.
  VectorXd set_point_vector = x0.head(num_dof);
  auto set_point = std::make_shared<
      AffineSystem<NullVector, NullVector, RigidBodySystem::StateVector>>(
      MatrixXd::Zero(0, 0), MatrixXd::Zero(0, 0), VectorXd::Zero(0),
      MatrixXd::Zero(num_dof, 0), MatrixXd::Zero(num_dof, 0), set_point_vector);

  auto controlled_robot = std::allocate_shared<
      GravityCompensatedPDPositionControlSystem<RigidBodySystem>>(
      Eigen::aligned_allocator<
          GravityCompensatedPDPositionControlSystem<RigidBodySystem>>(),
      iiwa_system, Kp, Kd);

  auto visualizer = CreateKukaIiwaVisualizer(iiwa_system);

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
  return drake::examples::kuka_iiwa_arm::DoMain(argc, argv);
}
