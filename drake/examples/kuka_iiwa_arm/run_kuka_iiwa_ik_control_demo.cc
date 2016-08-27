#include <iostream>

#include <lcm/lcm-cpp.hpp>

#include "drake/examples/kuka_iiwa_arm/iiwa_simulation.h"
#include "drake/examples/kuka_iiwa_arm/polynomial_trajectory_fit_generator.h"
#include "drake/systems/Simulation.h"
#include "drake/systems/cascade_system.h"
#include "drake/systems/controllers/open_loop_trajectory_controller.h"
#include "drake/systems/gravity_compensated_pd_position_control_system.h"
#include "drake/systems/plants/BotVisualizer.h"
#include "drake/systems/plants/RigidBodySystem.h"
#include "drake/systems/plants/RigidBodyTree.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using drake::BotVisualizer;
using drake::CascadeSystem;
using drake::GravityCompensatedPDPositionControlSystem;
using drake::OpenLoopTrajectoryController;
using drake::RigidBodySystem;
using lcm::LCM;

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {

int do_main(int argc, const char* argv[]) {
  std::shared_ptr<RigidBodySystem> iiwa_system = CreateKukaIiwaSystem(false);

  const auto& iiwa_tree = iiwa_system->getRigidBodyTree();

  MatrixXd joint_trajectories;
  std::vector<double> time_steps;

  GenerateIKDemoJointTrajectory(iiwa_tree, joint_trajectories, time_steps);
  PolynomialTrajectoryFitGenerator trajectory_runner(joint_trajectories,
                                                     time_steps);
  PiecewisePolynomial<double> pp =
      trajectory_runner.GenerateTrajectoryPolynomial();

  auto open_loop_planner =
      std::make_shared<OpenLoopTrajectoryController<RigidBodySystem>>(pp);
  int num_dof = iiwa_tree->number_of_positions();

  const double Kp_common = 7.5;    // Units : Nm/rad
  const double Kd_common = 0.075;  // Units : Nm/rad/sec
  VectorXd Kpdiag = VectorXd::Constant(num_dof, Kp_common);
  VectorXd Kddiag = VectorXd::Constant(num_dof, Kd_common);

  Kpdiag(1) = 10.0;
  Kpdiag(3) = 12.5;

  Kddiag(1) = 0.25;
  Kddiag(3) = 0.1;

  MatrixXd Kp = Kpdiag.asDiagonal();
  MatrixXd Kd = Kddiag.asDiagonal();

  auto controlled_robot = std::allocate_shared<
      GravityCompensatedPDPositionControlSystem<RigidBodySystem>>(
      Eigen::aligned_allocator<
          GravityCompensatedPDPositionControlSystem<RigidBodySystem>>(),
      iiwa_system, Kp, Kd);

  // Initializes LCM.
  std::shared_ptr<LCM> lcm = std::make_shared<LCM>();

  // Instantiates additional systems and cascades them with the rigid body
  // system.
  auto visualizer =
      std::make_shared<BotVisualizer<RigidBodySystem::StateVector>>(lcm,
                                                                    iiwa_tree);

  auto sys = cascade(cascade(open_loop_planner, controlled_robot), visualizer);

  // Simulation options
  const double kInitialStepSize = 0.001;
  const double kRealTimeFactor = 0.75;

  drake::SimulationOptions options =
      SetupSimulation(kInitialStepSize, kRealTimeFactor);

  // Specifies the start time of the simulation.
  const double kStartTime = 0;

  // Specifies the duration of the simulation.
  const double kDuration = 12.0;

  VectorXd x0 = VectorXd::Zero(iiwa_system->getNumStates());
  x0.head(num_dof) = iiwa_tree->getZeroConfiguration();
  drake::simulate(*sys.get(), kStartTime, kDuration, x0, options);

  return 0;
}

}  // namespace
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

int main(int argc, const char* argv[]) {
  return drake::examples::kuka_iiwa_arm::do_main(argc, argv);
}
