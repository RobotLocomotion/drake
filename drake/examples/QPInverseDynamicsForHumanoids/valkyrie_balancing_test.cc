#include "gtest/gtest.h"

#include "drake/common/drake_path.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/qp_controller.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/example_balancing_controller.h"
#include "drake/systems/plants/joints/floating_base_types.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {
// In this test, the Valkyrie robot is initialized to a nominal configuration
// with zero velocities, and the qp controller is setup to track this
// state. The robot is then perturbed in velocity for the Torso Pitch joint.
// The test forward simulates the closed loop system for 2 seconds.
// The simulation does not perform forward dynamics computation, instead, it
// integrates the computed acceleration from the controller. This dummy
// simulation should be replaced later with real simulation.
// The controller should drive the position and velocity close to zero in 2
// seconds.
GTEST_TEST(testQPInverseDynamicsController, testStanding) {
  // Loads model.
  std::string urdf =
      drake::GetDrakePath() +
      std::string(
          "/examples/Valkyrie/urdf/urdf/valkyrie_A_sim_drake_one_neck_dof_wide_ankle_rom.urdf");
  RigidBodyTree robot(urdf, drake::systems::plants::joints::kRollPitchYaw);
  HumanoidStatus robot_status(robot);

  QPController con;
  QPInput input(robot_status.robot());
  QPOutput output(robot_status.robot());

  // Setup initial condition.
  Eigen::VectorXd q(robot_status.robot().get_num_positions());
  Eigen::VectorXd v(robot_status.robot().get_num_velocities());

  q = robot_status.GetNominalPosition();
  v.setZero();
  Eigen::VectorXd q_ini = q;

  robot_status.Update(
      0, q, v, Eigen::VectorXd::Zero(robot_status.robot().actuators.size()),
      Eigen::Vector6d::Zero(), Eigen::Vector6d::Zero());

  // Setup a tracking problem.
  Eigen::Vector3d Kp_com = Eigen::Vector3d::Constant(40);
  Eigen::Vector3d Kd_com = Eigen::Vector3d::Constant(12);

  Eigen::Vector3d desired_com = robot_status.com();

  // Get an example controller that tracks a fixed point.
  input = MakeExampleQPInput(robot);
  TrackThis(robot_status, &input);

  // Perturb initial condition.
  q[robot_status.name_to_position_index().at("torsoPitch")] += 0.1;
  robot_status.Update(
      0, q, v, Eigen::VectorXd::Zero(robot_status.robot().actuators.size()),
      Eigen::Vector6d::Zero(), Eigen::Vector6d::Zero());

  // dt = 4e-3 is picked arbitrarily to ensure the test finishes within a
  // reasonable amount of time.
  double dt = 4e-3;
  double time = 0;

  // Feet should be stationary.
  EXPECT_TRUE(robot_status.foot(Side::LEFT).velocity().norm() < 1e-10);
  EXPECT_TRUE(robot_status.foot(Side::RIGHT).velocity().norm() < 1e-10);

  while (time < 2) {
    input.mutable_desired_comdd() =
      (Kp_com.array() * (desired_com - robot_status.com()).array() -
       Kd_com.array() * robot_status.comd().array()).matrix();
    int status = con.Control(robot_status, input, &output);

    if (status) {
      //std::cout << input;
      break;
    }

    // Dummy integration.
    // TODO(siyuan.feng@tri.gloabl): replace this with sys2 simulator when it's
    // ready.
    q += v * dt;
    v += output.vd() * dt;
    time += dt;

    //std::cout << "rs.q: " << robot_status.position().transpose() << std::endl;
    //std::cout << "rs.v: " << robot_status.velocity().transpose() << std::endl;
    //std::cout << input;
    //exit(0);

    robot_status.Update(time, q, v, output.joint_torque(),
                        Eigen::Vector6d::Zero(), Eigen::Vector6d::Zero());
    //std::cout << output;
  }

  // Check final state.
  // Since the feet have equality constraints set to 0 in the qp controller,
  // they should have no velocity after simulation.
  // Thus, the tolerances on feet velocities are smaller than those for the
  // generalized position and velocity.
  EXPECT_TRUE(robot_status.foot(Side::LEFT).velocity().norm() < 1e-6);
  EXPECT_TRUE(robot_status.foot(Side::RIGHT).velocity().norm() < 1e-6);

  EXPECT_TRUE(drake::CompareMatrices(q, q_ini, 1e-4,
                                     drake::MatrixCompareType::absolute));
  EXPECT_TRUE(drake::CompareMatrices(
      v, Eigen::VectorXd::Zero(robot_status.robot().get_num_velocities()), 1e-4,
      drake::MatrixCompareType::absolute));
}

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
