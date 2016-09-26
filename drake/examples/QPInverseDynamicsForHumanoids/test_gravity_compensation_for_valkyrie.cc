#include "drake/common/drake_path.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/qp_controller.h"
#include "drake/systems/plants/joints/floating_base_types.h"
#include "drake/math/rotation_matrix.h"
#include <eigen3/Eigen/Geometry>
#include <chrono>

QPOutput TestGravityCompensation(HumanoidStatus& robot_status) {
  // Make controller.
  QPController con(robot_status, 4);

  QPInput input(robot_status.robot());
  QPOutput output(robot_status.robot());

  // Make input.
  // These represent the desired motions for the robot, and are typically
  // outputs of motion planner or hand-crafted behavior state machines.
  input.mutable_comdd_d().setZero();
  input.mutable_pelvdd_d().setZero();
  input.mutable_torsodd_d().setZero();
  input.mutable_footdd_d(Side::LEFT).setZero();
  input.mutable_footdd_d(Side::RIGHT).setZero();
  input.mutable_vd_d().setZero();

  // Weights are set arbitrarily by the control designer, these typically
  // require tuning.
  input.mutable_w_com() = 1e2;
  input.mutable_w_pelv() = 1e1;
  input.mutable_w_torso() = 1e1;
  input.mutable_w_foot() = 1e5;
  input.mutable_w_vd() = 1e3;
  input.mutable_w_basis_reg() = 1e-5;

  // make contact
  input.mutable_supports().push_back(
      SupportElement(*robot_status.robot().FindBody("leftFoot")));
  input.mutable_support(0).get_mutable_contact_points().push_back(
      Vector3d(0.2, 0.05, -0.09));
  input.mutable_support(0).get_mutable_contact_points().push_back(
      Vector3d(0.2, -0.05, -0.09));
  input.mutable_support(0).get_mutable_contact_points().push_back(
      Vector3d(-0.05, -0.05, -0.09));
  input.mutable_support(0).get_mutable_contact_points().push_back(
      Vector3d(-0.05, 0.05, -0.09));

  input.mutable_supports().push_back(
      SupportElement(*robot_status.robot().FindBody("rightFoot")));
  input.mutable_support(1).get_mutable_contact_points().push_back(
      Vector3d(0.2, 0.05, -0.09));
  input.mutable_support(1).get_mutable_contact_points().push_back(
      Vector3d(0.2, -0.05, -0.09));
  input.mutable_support(1).get_mutable_contact_points().push_back(
      Vector3d(-0.05, -0.05, -0.09));
  input.mutable_support(1).get_mutable_contact_points().push_back(
      Vector3d(-0.05, 0.05, -0.09));

  con.Control(robot_status, input, &output);

  std::cout << output << std::endl;

  return output;
}

int main() {
  // Loads model.
  std::string urdf =
      drake::GetDrakePath() +
      std::string(
          "/examples/QPInverseDynamicsForHumanoids/valkyrie_sim_drake.urdf");
  RigidBodyTree robot(urdf, drake::systems::plants::joints::kRollPitchYaw);
  HumanoidStatus robot_status(robot);

  // Sets state and does kinematics.
  VectorXd q(robot_status.robot().get_num_positions());
  VectorXd v(robot_status.robot().get_num_velocities());

  q = robot_status.GetNominalPosition();
  v.setZero();

  robot_status.Update(0, q, v,
                      VectorXd::Zero(robot_status.robot().actuators.size()),
                      Vector6d::Zero(), Vector6d::Zero());

  QPOutput output = TestGravityCompensation(robot_status);

  robot_status.Update(0, q, v, output.joint_torque(),
                      output.foot_wrench_in_sensor_frame(Side::LEFT),
                      output.foot_wrench_in_sensor_frame(Side::RIGHT));
  return 0;
}
