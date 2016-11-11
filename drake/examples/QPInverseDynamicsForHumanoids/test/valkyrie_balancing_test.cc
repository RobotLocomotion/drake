#include "gtest/gtest.h"

#include "drake/common/drake_path.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/control_utils.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/example_qp_input_for_valkyrie.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/qp_controller.h"
#include "drake/multibody/joints/floating_base_types.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

// In this test, the Valkyrie robot is initialized to a nominal configuration
// with zero velocities, and the qp controller is set up to track this
// state. The robot is then perturbed in velocity for the Torso Pitch joint.
// The test forward simulates the closed loop system for 4 seconds.
// The simulation does not perform forward dynamics computation, instead, it
// integrates the computed acceleration from the controller. This dummy
// simulation should be replaced later with real simulation.
// The controller should drive the position and velocity close to zero in 4
// seconds.
GTEST_TEST(testQPInverseDynamicsController, testBalancingStanding) {
  // Loads model.
  std::string urdf =
      drake::GetDrakePath() +
      std::string(
          "/examples/Valkyrie/urdf/urdf/"
          "valkyrie_A_sim_drake_one_neck_dof_wide_ankle_rom.urdf");
  RigidBodyTree<double> robot(urdf,
                              drake::multibody::joints::kRollPitchYaw);
  HumanoidStatus robot_status(robot);

  QPController con;
  QPInput input = MakeExampleQPInput(robot_status);
  QPOutput output(robot);

  // Set up initial condition.
  VectorX<double> q(robot.get_num_positions());
  VectorX<double> v(robot.get_num_velocities());

  q = robot_status.GetNominalPosition();
  v.setZero();
  VectorX<double> q_ini = q;

  robot_status.Update(0, q, v, VectorX<double>::Zero(robot.actuators.size()),
                      Vector6<double>::Zero(), Vector6<double>::Zero());

  // Set up a tracking problem.
  Vector3<double> Kp_com = Vector3<double>::Constant(40);
  Vector3<double> Kd_com = Vector3<double>::Constant(12);

  Vector3<double> desired_com = robot_status.com();

  VectorX<double> Kp_q(VectorX<double>::Constant(q.size(), 20));
  VectorX<double> Kd_q(VectorX<double>::Constant(q.size(), 8));
  Kp_q.head<6>().setZero();
  Kd_q.head<6>().setZero();
  VectorSetpoint<double> joint_PDff(q, VectorX<double>::Zero(q.size()),
                                    VectorX<double>::Zero(q.size()), Kp_q,
                                    Kd_q);
  CartesianSetpoint<double> pelvis_PDff(
      robot_status.pelvis().pose(), Vector6<double>::Zero(),
      Vector6<double>::Zero(), Vector6<double>::Constant(20),
      Vector6<double>::Constant(8));
  CartesianSetpoint<double> torso_PDff(
      robot_status.torso().pose(), Vector6<double>::Zero(),
      Vector6<double>::Zero(), Vector6<double>::Constant(20),
      Vector6<double>::Constant(8));

  // Perturb initial condition.
  v[robot_status.name_to_position_index().at("torsoRoll")] += 1;
  robot_status.Update(0, q, v, VectorX<double>::Zero(robot.actuators.size()),
                      Vector6<double>::Zero(), Vector6<double>::Zero());

  // dt = 3e-3 is picked arbitrarily, with Gurobi, this one control call takes
  // roughly 3ms.
  double dt = 3e-3;
  double time = 0;

  // Feet should be stationary.
  EXPECT_TRUE(robot_status.foot(Side::LEFT).velocity().norm() < 1e-10);
  EXPECT_TRUE(robot_status.foot(Side::RIGHT).velocity().norm() < 1e-10);

  int tick_ctr = 0;
  while (time < 4) {
    // Update desired accelerations.
    input.mutable_desired_body_motions().at("pelvis").mutable_values() =
        pelvis_PDff.ComputeTargetAcceleration(robot_status.pelvis().pose(),
                                              robot_status.pelvis().velocity());
    input.mutable_desired_body_motions().at("torso").mutable_values() =
        torso_PDff.ComputeTargetAcceleration(robot_status.torso().pose(),
                                             robot_status.torso().velocity());
    input.mutable_desired_dof_motions().mutable_values() =
        joint_PDff.ComputeTargetAcceleration(robot_status.position(),
                                             robot_status.velocity());
    input.mutable_desired_centroidal_momentum_dot().mutable_values().tail<3>() =
        (Kp_com.array() * (desired_com - robot_status.com()).array() -
         Kd_com.array() * robot_status.comd().array()).matrix() *
        robot.getMass();

    int status = con.Control(robot_status, input, &output);

    if (status) {
      std::cout << input << output;
      break;
    }

    // Dummy integration.
    // TODO(siyuan.feng): replace this with sys2 simulator when it's ready.
    q += v * dt;
    v += output.vd() * dt;
    time += dt;

    robot_status.Update(time, q, v,
                        VectorX<double>::Zero(robot.actuators.size()),
                        Vector6<double>::Zero(), Vector6<double>::Zero());
    tick_ctr++;
  }

  // Check the final state.
  // Since the feet have equality constraints set to 0 in the qp controller,
  // they should have no velocity after simulation.
  // Thus, the tolerances on feet velocities are smaller than those for the
  // generalized position and velocity.
  EXPECT_TRUE(robot_status.foot(Side::LEFT).velocity().norm() < 1e-6);
  EXPECT_TRUE(robot_status.foot(Side::RIGHT).velocity().norm() < 1e-6);
  EXPECT_TRUE(drake::CompareMatrices(q, q_ini, 1e-4,
                                     drake::MatrixCompareType::absolute));
  EXPECT_TRUE(drake::CompareMatrices(
      v, VectorX<double>::Zero(robot.get_num_velocities()), 1e-4,
      drake::MatrixCompareType::absolute));

  std::cout << output;
}

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
