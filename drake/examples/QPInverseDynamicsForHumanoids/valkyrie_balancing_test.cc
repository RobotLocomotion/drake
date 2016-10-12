#include "gtest/gtest.h"

#include "drake/common/drake_path.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/qp_controller.h"
#include "drake/systems/plants/joints/floating_base_types.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

QPInput GenerateQPInput(
    const HumanoidStatus& robot_status, const Eigen::Vector3d& desired_com,
    const Eigen::Vector3d& Kp_com, const Eigen::Vector3d& Kd_com,
    const Eigen::VectorXd& desired_joints, const Eigen::VectorXd& Kp_joints,
    const Eigen::VectorXd& Kd_joints, const CartesianSetPoint& desired_pelvis,
    const CartesianSetPoint& desired_torso) {
  // Make input.
  QPInput input(robot_status.robot());

  // These represent the desired motions for the robot, and are typically
  // outputs of motion planner or hand-crafted behavior state machines.

  // Setup a PD tracking law for center of mass.
  input.mutable_desired_comdd() =
      (Kp_com.array() * (desired_com - robot_status.com()).array() -
       Kd_com.array() * robot_status.comd().array()).matrix();
  input.mutable_w_com() = 1e3;

  // Minimize acceleration in the generalized coordinates.
  input.mutable_desired_vd() =
      (Kp_joints.array() * (desired_joints - robot_status.position()).array() -
       Kd_joints.array() * robot_status.velocity().array()).matrix();
  input.mutable_w_vd() = 1;

  // Setup tracking for various body parts.
  DesiredBodyAcceleration pelvdd_d(*robot_status.robot().FindBody("pelvis"));
  pelvdd_d.mutable_weight() = 1e1;
  pelvdd_d.mutable_acceleration() = desired_pelvis.ComputeTargetAcceleration(
      robot_status.pelvis().pose(), robot_status.pelvis().velocity());
  input.mutable_desired_body_accelerations().push_back(pelvdd_d);

  DesiredBodyAcceleration torsodd_d(*robot_status.robot().FindBody("torso"));
  torsodd_d.mutable_weight() = 1e1;
  torsodd_d.mutable_acceleration() = desired_torso.ComputeTargetAcceleration(
      robot_status.torso().pose(), robot_status.torso().velocity());
  input.mutable_desired_body_accelerations().push_back(torsodd_d);

  // Weights are set arbitrarily by the control designer, these typically
  // require tuning.
  input.mutable_w_basis_reg() = 1e-6;

  // Make contact points.
  ContactInformation left_foot_contact(
      *robot_status.robot().FindBody("leftFoot"), 4);
  left_foot_contact.mutable_contact_points().push_back(
      Eigen::Vector3d(0.2, 0.05, -0.09));
  left_foot_contact.mutable_contact_points().push_back(
      Eigen::Vector3d(0.2, -0.05, -0.09));
  left_foot_contact.mutable_contact_points().push_back(
      Eigen::Vector3d(-0.05, -0.05, -0.09));
  left_foot_contact.mutable_contact_points().push_back(
      Eigen::Vector3d(-0.05, 0.05, -0.09));

  ContactInformation right_foot_contact(
      *robot_status.robot().FindBody("rightFoot"), 4);
  right_foot_contact.mutable_contact_points() =
      left_foot_contact.contact_points();

  input.mutable_contact_info().push_back(left_foot_contact);
  input.mutable_contact_info().push_back(right_foot_contact);

  return input;
}

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
          "/examples/QPInverseDynamicsForHumanoids/valkyrie_sim_drake.urdf");
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
  Eigen::VectorXd Kp_joints =
      Eigen::VectorXd::Constant(robot_status.robot().get_num_positions(), 20);
  Eigen::VectorXd Kd_joints =
      Eigen::VectorXd::Constant(robot_status.robot().get_num_velocities(), 8);
  Eigen::Vector6d Kp_pelvis = Eigen::Vector6d::Constant(20);
  Eigen::Vector6d Kd_pelvis = Eigen::Vector6d::Constant(8);
  Eigen::Vector6d Kp_torso = Eigen::Vector6d::Constant(20);
  Eigen::Vector6d Kd_torso = Eigen::Vector6d::Constant(8);

  Eigen::Vector3d desired_com = robot_status.com();
  Eigen::VectorXd desired_q = robot_status.position();
  CartesianSetPoint desired_pelvis(
      robot_status.pelvis().pose(), Eigen::Vector6d::Zero(),
      Eigen::Vector6d::Zero(), Kp_pelvis, Kd_pelvis);
  CartesianSetPoint desired_torso(robot_status.torso().pose(),
                                  Eigen::Vector6d::Zero(),
                                  Eigen::Vector6d::Zero(), Kp_torso, Kd_torso);

  // Perturb initial condition.
  v[robot_status.name_to_velocity_index().at("torsoPitch")] += 0.1;
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
    input =
        GenerateQPInput(robot_status, desired_com, Kp_com, Kd_com, desired_q,
                        Kp_joints, Kd_joints, desired_pelvis, desired_torso);
    int status = con.Control(robot_status, input, &output);

    if (status) break;

    // Dummy integration.
    // TODO(siyuan.feng@tri.gloabl): replace this with sys2 simulator when it's
    // ready.
    q += v * dt;
    v += output.vd() * dt;
    time += dt;

    robot_status.Update(time, q, v, output.joint_torque(),
                        Eigen::Vector6d::Zero(), Eigen::Vector6d::Zero());
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
