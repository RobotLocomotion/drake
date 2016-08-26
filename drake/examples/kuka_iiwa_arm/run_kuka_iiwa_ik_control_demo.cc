#include <iostream>

#include <lcm/lcm-cpp.hpp>

#include "drake/examples/kuka_iiwa_arm/iiwa_ik_trajectory_generator.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_simulation.h"
#include "drake/systems/plants/RigidBodySystem.h"
#include "drake/systems/plants/BotVisualizer.h"

#include "drake/common/drake_assert.h"
#include "drake/common/drake_path.h"
#include "drake/common/polynomial.h"
#include "drake/systems/controllers/open_loop_trajectory_controller.h"
#include "drake/systems/gravity_compensated_pd_position_control_system.h"
#include "drake/systems/cascade_system.h"
#include "drake/systems/Simulation.h"
#include "drake/systems/plants/IKoptions.h"
#include "drake/systems/plants/RigidBodyIK.h"
#include "drake/systems/plants/RigidBodyTree.h"
#include "drake/systems/plants/constraint/RigidBodyConstraint.h"
#include "drake/systems/trajectories/PiecewisePolynomial.h"
#include "drake/systems/vector.h"

#include "lcmtypes/drake/lcmt_iiwa_command.hpp"
#include "lcmtypes/drake/lcmt_iiwa_status.hpp"

#include "iiwa_status.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::VectorXi;
using drake::Vector1d;
using Eigen::Vector2d;
using Eigen::Vector3d;
using drake::RigidBodySystem;
using lcm::LCM;
using drake::BotVisualizer;
using drake::GravityCompensatedPDPositionControlSystem;
using drake::OpenLoopTrajectoryController;
using drake::CascadeSystem;

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {

void generateDemoConstraints(
    const std::shared_ptr<RigidBodyTree> iiwa_tree,
    std::vector<RigidBodyConstraint*> &constraint_array,
    std::vector<double> &time_stamps) {
  // Create a basic pointwise IK trajectory for moving the iiwa arm.
  // We start in the zero configuration (straight up).

  // TODO(sam.creasey) We should start planning with the robot's
  // current position rather than assuming vertical.
  VectorXd zero_conf = iiwa_tree->getZeroConfiguration();
  VectorXd joint_lb = zero_conf - VectorXd::Constant(7, 0.01);
  VectorXd joint_ub = zero_conf + VectorXd::Constant(7, 0.01);

  PostureConstraint pc1(iiwa_tree.get(), Vector2d(0, 0.5));
  VectorXi joint_idx(7);
  joint_idx << 0, 1, 2, 3, 4, 5, 6;
  pc1.setJointLimits(joint_idx, joint_lb, joint_ub);

  // Define an end effector constraint and make it active for the
  // timespan from 1 to 3 seconds.
  Vector3d pos_end;
  pos_end << 0.6, 0, 0.325;
  Vector3d pos_lb = pos_end - Vector3d::Constant(0.005);
  Vector3d pos_ub = pos_end + Vector3d::Constant(0.005);
  WorldPositionConstraint wpc(iiwa_tree.get(), iiwa_tree->FindBodyIndex("iiwa_link_ee"),
                              Vector3d::Zero(), pos_lb, pos_ub, Vector2d(1, 3));

  // After the end effector constraint is released, apply the straight
  // up configuration again.
  PostureConstraint pc2(iiwa_tree.get(), Vector2d(4, 5.9));
  pc2.setJointLimits(joint_idx, joint_lb, joint_ub);

  // Bring back the end effector constraint through second 9 of the
  // demo.
  WorldPositionConstraint wpc2(iiwa_tree.get(), iiwa_tree->FindBodyIndex("iiwa_link_ee"),
                               Vector3d::Zero(), pos_lb, pos_ub,
                               Vector2d(6, 9));

  // For part of the remaining time, constrain the second joint while
  // preserving the end effector constraint.
  //
  // Variable `joint_position_start_idx` below is a collection of offsets into
  // the state vector referring to the positions of the joints to be
  // constrained.
  Eigen::VectorXi joint_position_start_idx(1);
  joint_position_start_idx(0) = iiwa_tree->FindChildBodyOfJoint("iiwa_joint_2")->
      get_position_start_index();
  PostureConstraint pc3(iiwa_tree.get(), Vector2d(6, 8));
  pc3.setJointLimits(joint_position_start_idx, Vector1d(0.7), Vector1d(0.8));

  time_stamps.push_back(0.0);
  time_stamps.push_back(2.0);
  time_stamps.push_back(5.0);
  time_stamps.push_back(7.0);
  time_stamps.push_back(9.0);
//
//  const int kNumTimesteps = 5;
//  double t[kNumTimesteps] = { 0.0, 2.0, 5.0, 7.0, 9.0 };
//  MatrixXd q0(iiwa_tree->number_of_positions(), kNumTimesteps);
//  for (int i = 0; i < kNumTimesteps; i++) {
//    q0.col(i) = zero_conf;
//  }
//
//  std::vector<RigidBodyConstraint*> constraint_array;
  constraint_array.push_back(&pc1);
  constraint_array.push_back(&wpc);
  constraint_array.push_back(&pc2);
  constraint_array.push_back(&pc3);
  constraint_array.push_back(&wpc2);

  //return(constraint_array);
}


int do_main(int argc, const char* argv[]) {

  std::shared_ptr<RigidBodySystem> iiwa_system = CreateKukaIiwaSystem();

  const auto& iiwa_tree = iiwa_system->getRigidBodyTree();

  // Initializes LCM.
  std::shared_ptr<LCM> lcm = std::make_shared<LCM>();

  // Instantiates additional systems and cascades them with the rigid body
  // system.
  auto visualizer =
      std::make_shared<BotVisualizer<RigidBodySystem::StateVector>>(lcm,
          iiwa_tree);

  std::vector<RigidBodyConstraint*> demo_constraints;
  std::vector<double> time_steps;
  generateDemoConstraints(iiwa_tree, demo_constraints, time_steps);

  std::cout<<"So far so good\n\n";
  TrajectoryGenerator trajectory_runner(demo_constraints, time_steps, iiwa_tree);

  PiecewisePolynomial<double> pp = trajectory_runner.GenerateTrajectoryPolynomial();
  // Now run through the plan.
//  ?TrajectoryGenerator runner(lcm, kNumTimesteps, t, q_sol);
//  runner.Run();


  auto open_loop_planner = std::make_shared<
      OpenLoopTrajectoryController<RigidBodySystem>>(pp);
  int num_dof = iiwa_tree->number_of_positions();

  // Smaller gains intentionally used for demo.
  const double Kp_common = 10.0;  // Units : Nm/rad
  const double Kd_common = 0.30;  // Units : Nm/rad/sec
  VectorXd Kpdiag = VectorXd::Constant(num_dof, Kp_common);
  VectorXd Kddiag = VectorXd::Constant(num_dof, Kd_common);

  MatrixXd Kp = Kpdiag.asDiagonal();
  MatrixXd Kd = Kddiag.asDiagonal();


  auto controlled_robot = std::allocate_shared<
  GravityCompensatedPDPositionControlSystem<RigidBodySystem>>(
      Eigen::aligned_allocator<
          GravityCompensatedPDPositionControlSystem<RigidBodySystem>>(),
      iiwa_system, Kp, Kd);
//

//
  auto sys = cascade(cascade(open_loop_planner, controlled_robot), visualizer);
//
  drake::SimulationOptions options = SetupSimulation();

  // Specifies the start time of the simulation.
  const double kStartTime = 0;

  // Specifies the duration of the simulation.
  const double kDuration = 10.0;
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
