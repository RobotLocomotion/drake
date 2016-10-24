#include "drake/examples/QPInverseDynamicsForHumanoids/example_balancing_controller.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

void TrackThis(const HumanoidStatus& rs, QPInput* input) {
  int dim = rs.robot().get_num_velocities();
  input->mutable_desired_joint_motions().mutable_setpoint() =
      VectorSetpoint(rs.position(),
                     Eigen::VectorXd::Zero(dim),
                     Eigen::VectorXd::Zero(dim),
                     Eigen::VectorXd::Constant(dim, 20),
                     Eigen::VectorXd::Constant(dim, 8));

  for (DesiredBodyMotion& body_motion : input->mutable_desired_body_motions()) {
    const RigidBody& body = body_motion.body();
    Eigen::Isometry3d pose_d = rs.robot().relativeTransform(rs.cache(), 0, body.get_body_index());
    body_motion.mutable_setpoint() =
      CartesianSetpoint(pose_d,
                        Eigen::Vector6d::Zero(),
                        Eigen::Vector6d::Zero(),
                        Eigen::Vector6d::Constant(20),
                        Eigen::Vector6d::Constant(8));
  }
}

QPInput MakeExampleQPInput(const RigidBodyTree& robot) {
  int dim = robot.get_num_velocities();
  QPInput input(robot);

  // Setup a PD tracking law for center of mass.
  input.mutable_desired_comdd() = Eigen::Vector3d::Zero();
  input.mutable_w_com() = 1e3;

  // Minimize acceleration in the generalized coordinates.
  input.mutable_desired_joint_motions().mutable_weights() = Eigen::VectorXd::Constant(dim, 1e-2);

  // Setup tracking for various body parts.
  DesiredBodyMotion pelvdd_d(*robot.FindBody("pelvis"));
  pelvdd_d.mutable_weights() = Eigen::Vector6d::Constant(-1e1);
  pelvdd_d.mutable_weights().segment<3>(3).setZero();
  input.mutable_desired_body_motions().push_back(pelvdd_d);

  DesiredBodyMotion torsodd_d(*robot.FindBody("torso"));
  torsodd_d.mutable_weights() = Eigen::Vector6d::Constant(-1e1);
  torsodd_d.mutable_weights().segment<3>(3).setZero();
  input.mutable_desired_body_motions().push_back(torsodd_d);

  // Weights are set arbitrarily by the control designer, these typically
  // require tuning.
  input.mutable_w_basis_reg() = 1e-6;

  // Make contact points.
  ContactInformation left_foot_contact(
      *robot.FindBody("leftFoot"), 4);
  left_foot_contact.mutable_contact_points().push_back(
      Eigen::Vector3d(0.2, 0.05, -0.09));
  left_foot_contact.mutable_contact_points().push_back(
      Eigen::Vector3d(0.2, -0.05, -0.09));
  left_foot_contact.mutable_contact_points().push_back(
      Eigen::Vector3d(-0.05, -0.05, -0.09));
  left_foot_contact.mutable_contact_points().push_back(
      Eigen::Vector3d(-0.05, 0.05, -0.09));

  ContactInformation right_foot_contact(
      *robot.FindBody("rightFoot"), 4);
  right_foot_contact.mutable_contact_points() =
      left_foot_contact.contact_points();

  input.mutable_contact_info().push_back(left_foot_contact);
  input.mutable_contact_info().push_back(right_foot_contact);

  return input;
}

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
