#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/text_logging.h"
#include "drake/examples/valkyrie/valkyrie_constants.h"
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

using examples::valkyrie::kRPYValkyrieDof;
using examples::valkyrie::RPYValkyrieFixedPointState;

Isometry3<double> ComputeBodyPose(const RobotKinematicState<double>& status,
    const RigidBody<double>& body) {
  const RigidBodyTree<double>& robot = status.get_robot();
  return robot.CalcBodyPoseInWorldFrame(status.get_cache(), body);
}

Vector6<double> ComputeBodyVelocity(const RobotKinematicState<double>& status,
    const RigidBody<double>& body) {
  const RigidBodyTree<double>& robot = status.get_robot();
  return robot.CalcBodySpatialVelocityInWorldFrame(status.get_cache(), body);
}

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
  std::string urdf = FindResourceOrThrow(
      "drake/examples/valkyrie/urdf/urdf/"
      "valkyrie_A_sim_drake_one_neck_dof_wide_ankle_rom.urdf");
  std::string alias_groups_config = FindResourceOrThrow(
      "drake/systems/controllers/qp_inverse_dynamics/test/"
      "valkyrie.alias_groups");
  std::string controller_config = FindResourceOrThrow(
      "drake/systems/controllers/qp_inverse_dynamics/test/"
      "valkyrie.id_controller_config");

  RigidBodyTree<double> robot;
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      urdf, multibody::joints::kRollPitchYaw, &robot);

  // KinematicsProperty
  RigidBodyTreeAliasGroups<double> alias_groups(&robot);
  alias_groups.LoadFromFile(alias_groups_config);

  // Controller config
  ParamSet paramset;
  paramset.LoadFromFile(controller_config, alias_groups);

  RobotKinematicState<double> robot_status(&robot);

  const RigidBody<double>& pelvis = *alias_groups.get_body("pelvis");
  const RigidBody<double>& torso = *alias_groups.get_body("torso");
  const RigidBody<double>& left_foot = *alias_groups.get_body("left_foot");
  const RigidBody<double>& right_foot = *alias_groups.get_body("right_foot");

  QpInverseDynamics con;
  QpInput input = paramset.MakeQpInput({"feet"},            /* contacts */
                                       {"pelvis", "torso"}, /* tracked bodies*/
                                       alias_groups);
  QpOutput output(
      systems::controllers::qp_inverse_dynamics::GetDofNames(robot));

  // Set up initial condition.
  DRAKE_DEMAND(kRPYValkyrieDof == robot.get_num_positions());
  VectorX<double> q = RPYValkyrieFixedPointState().head(kRPYValkyrieDof);
  VectorX<double> v = VectorX<double>::Zero(robot.get_num_velocities());
  VectorX<double> q_ini = q;

  robot_status.UpdateKinematics(0, q, v);

  // Set up a tracking problem.
  // Gains
  Vector3<double> Kp_com, Kd_com;
  VectorX<double> Kp_q, Kd_q;
  Vector6<double> Kp_pelvis, Kp_torso, Kd_pelvis, Kd_torso, Kp_centroidal,
      Kd_centroidal;

  paramset.LookupDesiredBodyMotionGains(pelvis, &Kp_pelvis, &Kd_pelvis);
  paramset.LookupDesiredBodyMotionGains(torso, &Kp_torso, &Kd_torso);
  paramset.LookupDesiredDofMotionGains(&Kp_q, &Kd_q);
  paramset.LookupDesiredCentroidalMomentumDotGains(&Kp_centroidal,
                                                   &Kd_centroidal);
  Kp_com = Kp_centroidal.tail<3>();
  Kd_com = Kd_centroidal.tail<3>();

  // Setpoints
  Vector3<double> desired_com = robot_status.get_com();
  VectorSetpoint<double> joint_PDff(q, VectorX<double>::Zero(q.size()),
                                    VectorX<double>::Zero(q.size()), Kp_q,
                                    Kd_q);
  auto pelvis_pose = ComputeBodyPose(robot_status, pelvis);
  auto pelvis_vel = ComputeBodyVelocity(robot_status, pelvis);
  auto torso_pose = ComputeBodyPose(robot_status, torso);
  auto torso_vel = ComputeBodyVelocity(robot_status, torso);
  CartesianSetpoint<double> pelvis_PDff(
      pelvis_pose, Vector6<double>::Zero(),
      Vector6<double>::Zero(), Kp_pelvis, Kd_pelvis);
  CartesianSetpoint<double> torso_PDff(
      torso_pose, Vector6<double>::Zero(),
      Vector6<double>::Zero(), Kp_torso, Kd_torso);

  // Perturb initial condition.
  v[alias_groups.get_velocity_group("back_x").front()] += 1;
  robot_status.UpdateKinematics(0, q, v);
  // Need to update torso velocity again since back_x joint has velocity now.
  torso_vel = ComputeBodyVelocity(robot_status, torso);

  // dt = 3e-3 is picked arbitrarily, with Gurobi, this one control call takes
  // roughly 3ms.
  double dt = 3e-3;
  double time = 0;

  // Feet should be stationary.
  auto left_foot_vel = ComputeBodyVelocity(robot_status, left_foot);
  auto right_foot_vel = ComputeBodyVelocity(robot_status, right_foot);
  EXPECT_TRUE(left_foot_vel.norm() < 1e-10);
  EXPECT_TRUE(right_foot_vel.norm() < 1e-10);

  int tick_ctr = 0;
  const std::string& pelvis_body_name = pelvis.get_name();
  const std::string& torso_body_name = torso.get_name();

  while (time < 4) {
    // Update desired accelerations.
    input.mutable_desired_body_motions().at(pelvis_body_name).mutable_values() =
        pelvis_PDff.ComputeTargetAcceleration(pelvis_pose, pelvis_vel);
    input.mutable_desired_body_motions().at(torso_body_name).mutable_values() =
        torso_PDff.ComputeTargetAcceleration(torso_pose, torso_vel);

    input.mutable_desired_dof_motions().mutable_values() =
        joint_PDff.ComputeTargetAcceleration(robot_status.get_cache().getQ(),
                                             robot_status.get_cache().getV());
    input.mutable_desired_centroidal_momentum_dot().mutable_values().tail<3>() =
        (Kp_com.array() * (desired_com - robot_status.get_com()).array() -
         Kd_com.array() * robot_status.get_com_velocity().array())
            .matrix() *
        robot.getMass();

    int status = con.Control(robot_status, input, &output);

    if (status) {
      std::stringstream err;
      err << input << output;
      drake::log()->info(err.str());
      break;
    }

    // Dummy integration.
    // TODO(siyuan.feng): replace this with sys2 simulator when it's ready.
    q += v * dt;
    v += output.vd() * dt;
    time += dt;

    robot_status.UpdateKinematics(time, q, v);
    pelvis_pose = ComputeBodyPose(robot_status, pelvis);
    pelvis_vel = ComputeBodyVelocity(robot_status, pelvis);
    torso_pose = ComputeBodyPose(robot_status, torso);
    torso_vel = ComputeBodyVelocity(robot_status, torso);
    left_foot_vel = ComputeBodyVelocity(robot_status, left_foot);
    right_foot_vel = ComputeBodyVelocity(robot_status, right_foot);

    tick_ctr++;
  }

  // Check the final state.
  // Since the feet have equality constraints set to 0 in the qp controller,
  // they should have no velocity after simulation.
  // Thus, the tolerances on feet velocities are smaller than those for the
  // generalized position and velocity.
  EXPECT_TRUE(left_foot_vel.norm() < 1e-6);
  EXPECT_TRUE(right_foot_vel.norm() < 1e-6);
  EXPECT_TRUE(drake::CompareMatrices(q, q_ini, 1e-3,
                                     drake::MatrixCompareType::absolute));
  EXPECT_TRUE(drake::CompareMatrices(
      v, VectorX<double>::Zero(robot.get_num_velocities()), 1e-4,
      drake::MatrixCompareType::absolute));
}

}  // namespace
}  // namespace qp_inverse_dynamics
}  // namespace controllers
}  // namespace systems
}  // namespace drake
