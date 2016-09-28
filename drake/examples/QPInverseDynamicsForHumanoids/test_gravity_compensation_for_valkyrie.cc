#include "drake/common/drake_path.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/qp_controller.h"
#include "drake/systems/plants/joints/floating_base_types.h"
#include "drake/math/rotation_matrix.h"
#include <eigen3/Eigen/Geometry>
#include <chrono>

QPOutput TestGravityCompensation(const HumanoidStatus& robot_status) {
  // Make controller.
  QPController con;

  QPInput input(robot_status.robot());
  QPOutput output(robot_status.robot());

  // Make input.
  // These represent the desired motions for the robot, and are typically
  // outputs of motion planner or hand-crafted behavior state machines.
  input.mutable_desired_comdd().setZero();
  input.mutable_desired_vd().setZero();
  DesiredBodyAcceleration pelvdd_d(*robot_status.robot().FindBody("pelvis"));
  pelvdd_d.mutable_weight() = 1e1;
  pelvdd_d.mutable_acceleration().setZero();

  DesiredBodyAcceleration torsodd_d(*robot_status.robot().FindBody("torso"));
  torsodd_d.mutable_weight() = 1e1;
  torsodd_d.mutable_acceleration().setZero();

  DesiredBodyAcceleration left_footdd_d(
      *robot_status.robot().FindBody("leftFoot"));
  left_footdd_d.mutable_weight() = 1e3;
  left_footdd_d.mutable_acceleration().setZero();

  DesiredBodyAcceleration right_footdd_d(
      *robot_status.robot().FindBody("rightFoot"));
  right_footdd_d.mutable_weight() = 1e3;
  right_footdd_d.mutable_acceleration().setZero();

  input.mutable_desired_body_accelerations().push_back(pelvdd_d);
  input.mutable_desired_body_accelerations().push_back(torsodd_d);
  input.mutable_desired_body_accelerations().push_back(left_footdd_d);
  input.mutable_desired_body_accelerations().push_back(right_footdd_d);

  // Weights are set arbitrarily by the control designer, these typically
  // require tuning.
  input.mutable_w_com() = 1e2;
  input.mutable_w_vd() = 1e-1;
  input.mutable_w_basis_reg() = 1e-6;

  // Make contact points.
  ContactInformation left_foot_contact(
      *robot_status.robot().FindBody("leftFoot"), 4);
  left_foot_contact.mutable_contact_points().push_back(
      Vector3d(0.2, 0.05, -0.09));
  left_foot_contact.mutable_contact_points().push_back(
      Vector3d(0.2, -0.05, -0.09));
  left_foot_contact.mutable_contact_points().push_back(
      Vector3d(-0.05, -0.05, -0.09));
  left_foot_contact.mutable_contact_points().push_back(
      Vector3d(-0.05, 0.05, -0.09));

  ContactInformation right_foot_contact(
      *robot_status.robot().FindBody("rightFoot"), 4);
  right_foot_contact.mutable_contact_points() =
      left_foot_contact.contact_points();

  input.mutable_contact_info().push_back(left_foot_contact);
  input.mutable_contact_info().push_back(right_foot_contact);

  std::cout << input << std::endl;

  // Call controller.
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
  return 0;
}
