#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/text_logging.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/systems/controllers/qp_inverse_dynamics/param_parser.h"
#include "drake/systems/controllers/qp_inverse_dynamics/qp_inverse_dynamics.h"
#include "drake/systems/controllers/setpoint.h"

namespace drake {
namespace systems {
namespace controllers {
namespace qp_inverse_dynamics {
namespace {

// Sets up a inverse dynamics problem for a fixed based manipulator (iiwa arm).
// The only objective is to only track acceleration in the generalized
// coordinate with no contacts. If not limited by joint torque, inverse
// dynamics should return a generalized acceleration very close to the desired
// one.
GTEST_TEST(testQPInverseDynamicsController, testForIiwa) {
  std::string urdf = FindResourceOrThrow(
      "drake/manipulation/models/iiwa_description/urdf/"
      "iiwa14_polytope_collision.urdf");
  std::string alias_groups_config = FindResourceOrThrow(
      "drake/systems/controllers/qp_inverse_dynamics/test/"
      "iiwa.alias_groups");
  std::string controller_config = FindResourceOrThrow(
      "drake/systems/controllers/qp_inverse_dynamics/test/"
      "iiwa.id_controller_config");

  RigidBodyTree<double> robot;
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      urdf, multibody::joints::kFixed, &robot);

  RigidBodyTreeAliasGroups<double> alias_groups(&robot);
  alias_groups.LoadFromFile(alias_groups_config);

  ParamSet paramset;
  paramset.LoadFromFile(controller_config, alias_groups);

  RobotKinematicState<double> robot_status(&robot);

  QpInverseDynamics con;
  std::vector<std::string> contact_group_names = {};
  std::vector<std::string> tracked_body_names = {};
  QpInput input = paramset.MakeQpInput(contact_group_names, tracked_body_names,
                                       alias_groups);
  QpOutput output(GetDofNames(robot));

  // Sets up desired q and v.
  VectorX<double> q = VectorX<double>::Zero(robot.get_num_positions());
  VectorX<double> v = VectorX<double>::Zero(robot.get_num_velocities());

  // Sets up a control policy to track q and v.
  VectorX<double> kp, kd;
  paramset.LookupDesiredDofMotionGains(&kp, &kd);
  VectorSetpoint<double> policy(q, v, VectorX<double>::Zero(q.size()), kp, kd);

  // Perturbs the initial condition.
  v[1] += 1;
  robot_status.UpdateKinematics(0, q, v);

  // Uses policy to generate desired acceleration.
  input.mutable_desired_dof_motions().mutable_values() =
      policy.ComputeTargetAcceleration(q, v);

  // Calls inverse dynamics controller.
  int status = con.Control(robot_status, input, &output);
  EXPECT_EQ(status, 0);

  // The output acceleration should match the desired very closely with no
  // constraints and other competing costs.
  EXPECT_TRUE(drake::CompareMatrices(input.desired_dof_motions().values(),
                                     output.vd(), 1e-9,
                                     drake::MatrixCompareType::absolute));

  // Without any external forces or hitting any contraints, torque = M * vd_d +
  // h.
  VectorX<double> expected_torque =
      robot_status.get_M() * input.desired_dof_motions().values() +
      robot_status.get_bias_term();
  EXPECT_TRUE(drake::CompareMatrices(expected_torque, output.dof_torques(),
                                     1e-9, drake::MatrixCompareType::absolute));
}

}  // namespace
}  // namespace qp_inverse_dynamics
}  // namespace controllers
}  // namespace systems
}  // namespace drake
