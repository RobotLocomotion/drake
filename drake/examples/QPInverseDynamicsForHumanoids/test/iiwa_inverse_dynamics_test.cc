#include <gtest/gtest.h>

#include "drake/common/drake_path.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/common/text_logging.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/control_utils.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/param_parsers/param_parser.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/qp_controller.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

// Sets up a inverse dynamics problem for a fixed based manipulator (iiwa arm).
// The only objective is to only track acceleration in the generalized
// coordinate with no contacts. If not limited by joint torque, inverse
// dynamics should return a generalized acceleration very close to the desired
// one.
GTEST_TEST(testQPInverseDynamicsController, testForIiwa) {
  std::string urdf = drake::GetDrakePath() +
                     "/examples/kuka_iiwa_arm/models/iiwa14/"
                     "iiwa14_simplified_collision.urdf";
  std::string alias_groups_config = drake::GetDrakePath() +
                                    "/examples/QPInverseDynamicsForHumanoids/"
                                    "config/iiwa.alias_groups";
  std::string controller_config = drake::GetDrakePath() +
                                  "/examples/QPInverseDynamicsForHumanoids/"
                                  "config/iiwa.id_controller_config";

  auto robot = std::make_unique<RigidBodyTree<double>>();
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      urdf, multibody::joints::kFixed, robot.get());

  param_parsers::RigidBodyTreeAliasGroups<double> alias_groups(*robot);
  alias_groups.LoadFromFile(alias_groups_config);

  param_parsers::ParamSet paramset;
  paramset.LoadFromFile(controller_config, alias_groups);

  HumanoidStatus robot_status(*robot, alias_groups);

  QPController con;
  QpInput input = paramset.MakeQpInput({}, /* contacts */
                                       {}, /* tracked bodies*/
                                       alias_groups);
  QpOutput output(GetDofNames(*robot));

  // Sets up desired q and v.
  VectorX<double> q = VectorX<double>::Zero(robot->get_num_positions());
  VectorX<double> v = VectorX<double>::Zero(robot->get_num_velocities());

  // Sets up a control policy to track q and v.
  VectorX<double> kp, kd;
  paramset.LookupDesiredDofMotionGains(&kp, &kd);
  VectorSetpoint<double> policy(q, v, VectorX<double>::Zero(q.size()), kp, kd);

  // Perturbs the initial condition.
  v[1] += 1;
  robot_status.Update(0, q, v,
                      VectorX<double>::Zero(robot->get_num_actuators()),
                      Vector6<double>::Zero(), Vector6<double>::Zero());

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
      robot_status.M() * input.desired_dof_motions().values() +
      robot_status.bias_term();
  EXPECT_TRUE(drake::CompareMatrices(expected_torque, output.dof_torques(),
                                     1e-9, drake::MatrixCompareType::absolute));
}

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
