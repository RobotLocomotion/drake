#include "drake/examples/QPInverseDynamicsForHumanoids/example_qp_input_for_valkyrie.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

QPInput MakeExampleQPInput(const RigidBodyTree<double>& robot) {
  int dim = robot.get_num_velocities();
  QPInput input(robot);

  // Set up a PD tracking law for center of mass.
  input.mutable_desired_centroidal_momentum_dot().mutable_weights() =
      Eigen::Vector6d::Constant(1);
  // Wipe out the weights for the angular part.
  input.mutable_desired_centroidal_momentum_dot()
      .mutable_weights()
      .head<3>()
      .setZero();
  input.mutable_desired_centroidal_momentum_dot().SetConstraintType(
      {3, 4, 5}, ConstraintType::Soft);

  // Minimize acceleration in the generalized coordinates.
  input.mutable_desired_joint_motions().mutable_weights() =
      Eigen::VectorXd::Constant(dim, 1e-2);
  input.mutable_desired_joint_motions().SetAllConstraintType(
      ConstraintType::Soft);

  // Set up tracking for various body parts.
  DesiredBodyMotion pelvdd_d(*robot.FindBody("pelvis"));
  pelvdd_d.mutable_weights().head<3>() = Eigen::Vector3d(1, -1, 1);
  pelvdd_d.SetConstraintType({0, 2}, ConstraintType::Soft);
  pelvdd_d.SetConstraintType({1}, ConstraintType::Hard);
  // Wipe out the weights for the position part.
  input.mutable_desired_body_motions().emplace(pelvdd_d.name(), pelvdd_d);

  DesiredBodyMotion torsodd_d(*robot.FindBody("torso"));
  torsodd_d.mutable_weights().head<3>() = Eigen::Vector3d(-1, 1, 1);
  torsodd_d.SetConstraintType({1, 2}, ConstraintType::Soft);
  torsodd_d.SetConstraintType({0}, ConstraintType::Hard);
  // Wipe out the weights for the position part.
  input.mutable_desired_body_motions().emplace(torsodd_d.name(), torsodd_d);

  // Weights are set arbitrarily by the control designer, these typically
  // require tuning.
  input.mutable_w_basis_reg() = 1e-6;

  // Make contact points.
  ContactInformation left_foot_contact(*robot.FindBody("leftFoot"));
  left_foot_contact.mutable_contact_points().push_back(
      Eigen::Vector3d(0.2, 0.05, -0.09));
  left_foot_contact.mutable_contact_points().push_back(
      Eigen::Vector3d(0.2, -0.05, -0.09));
  left_foot_contact.mutable_contact_points().push_back(
      Eigen::Vector3d(-0.05, -0.05, -0.09));
  left_foot_contact.mutable_contact_points().push_back(
      Eigen::Vector3d(-0.05, 0.05, -0.09));
  left_foot_contact.mutable_acceleration_constraint_type() =
      ConstraintType::Soft;
  // Deliberately set left and right foot's stationary conditions to soft
  // and hard for testing.
  left_foot_contact.mutable_weight() = 1e5;
  left_foot_contact.mutable_Kd() = 8;

  ContactInformation right_foot_contact(*robot.FindBody("rightFoot"));
  right_foot_contact.mutable_contact_points() =
      left_foot_contact.contact_points();
  right_foot_contact.mutable_acceleration_constraint_type() =
      ConstraintType::Hard;
  right_foot_contact.mutable_weight() = -1;
  right_foot_contact.mutable_Kd() = 8;

  input.mutable_contact_info().push_back(left_foot_contact);
  input.mutable_contact_info().push_back(right_foot_contact);

  return input;
}

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
