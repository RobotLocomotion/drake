#include "gtest/gtest.h"

#include "drake/common/drake_path.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/common/text_logging.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/control_utils.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/param_parsers/param_parser.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/qp_controller.h"
#include "drake/examples/Valkyrie/valkyrie_constants.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"

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
  std::string urdf = drake::GetDrakePath() + "/examples/Valkyrie/urdf/urdf/"
      "valkyrie_A_sim_drake_one_neck_dof_wide_ankle_rom.urdf";
  std::string alias_groups_config =
      drake::GetDrakePath() + "/examples/QPInverseDynamicsForHumanoids/"
          "config/alias_groups.yaml";
  std::string controller_config =
      drake::GetDrakePath() + "/examples/QPInverseDynamicsForHumanoids/"
          "config/controller.yaml";

  auto robot = std::make_unique<RigidBodyTree<double>>();
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      urdf, multibody::joints::kRollPitchYaw, robot.get());

  // KinematicsProperty
  param_parsers::RigidBodyTreeAliasGroups<double> alias_groups(*robot);
  alias_groups.LoadFromYAMLFile(YAML::LoadFile(alias_groups_config));

  // Controller config
  param_parsers::ParamSet paramset;
  paramset.LoadFromYAMLConfigFile(YAML::LoadFile(controller_config),
                                  alias_groups);

  HumanoidStatus robot_status(*robot, alias_groups);

  QPController con;
  QpInput input(GetDofNames(*robot));
  QpOutput output(GetDofNames(*robot));

  // Initializes QP input.
  input.mutable_contact_information() =
      paramset.MakeContactInformation("feet", alias_groups);

  std::unordered_map<std::string, DesiredBodyMotion> motion_d =
      paramset.MakeDesiredBodyMotion("pelvis", alias_groups);
  input.mutable_desired_body_motions().insert(motion_d.begin(), motion_d.end());
  motion_d = paramset.MakeDesiredBodyMotion("torso", alias_groups);
  input.mutable_desired_body_motions().insert(motion_d.begin(), motion_d.end());

  input.mutable_desired_dof_motions() = paramset.MakeDesiredDofMotions();
  input.mutable_w_basis_reg() = paramset.get_basis_regularization_weight();

  input.mutable_desired_centroidal_momentum_dot() =
      paramset.MakeDesiredCentroidalMomentumDot();

  // Set up initial condition.
  DRAKE_DEMAND(valkyrie::kRPYValkyrieDof == robot->get_num_positions());
  VectorX<double> q =
      valkyrie::RPYValkyrieFixedPointState().head(valkyrie::kRPYValkyrieDof);
  VectorX<double> v = VectorX<double>::Zero(robot->get_num_velocities());
  VectorX<double> q_ini = q;

  robot_status.Update(0, q, v,
      VectorX<double>::Zero(robot->get_num_actuators()),
      Vector6<double>::Zero(), Vector6<double>::Zero());

  // Set up a tracking problem.
  // Gains
  Vector3<double> Kp_com, Kd_com;
  VectorX<double> Kp_q, Kd_q;
  Vector6<double> Kp_pelvis, Kp_torso, Kd_pelvis, Kd_torso, Kp_centroidal,
      Kd_centroidal;

  paramset.LookupDesiredBodyMotionGains(*robot->FindBody("pelvis"), &Kp_pelvis,
                                        &Kd_pelvis);
  paramset.LookupDesiredBodyMotionGains(*robot->FindBody("torso"), &Kp_torso,
                                        &Kd_torso);
  paramset.LookupDesiredDofMotionGains(&Kp_q, &Kd_q);
  paramset.LookupDesiredCentroidalMomentumDotGains(&Kp_centroidal,
                                                   &Kd_centroidal);
  Kp_com = Kp_centroidal.tail<3>();
  Kd_com = Kd_centroidal.tail<3>();

  // Setpoints
  Vector3<double> desired_com = robot_status.com();
  VectorSetpoint<double> joint_PDff(q, VectorX<double>::Zero(q.size()),
                                    VectorX<double>::Zero(q.size()), Kp_q,
                                    Kd_q);
  CartesianSetpoint<double> pelvis_PDff(
      robot_status.pelvis().pose(), Vector6<double>::Zero(),
      Vector6<double>::Zero(), Kp_pelvis, Kd_pelvis);
  CartesianSetpoint<double> torso_PDff(
      robot_status.torso().pose(), Vector6<double>::Zero(),
      Vector6<double>::Zero(), Kp_torso, Kd_torso);

  // Perturb initial condition.
  v[robot_status.name_to_position_index().at("torsoRoll")] += 1;
  robot_status.Update(0, q, v,
      VectorX<double>::Zero(robot->get_num_actuators()),
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
         Kd_com.array() * robot_status.comd().array())
            .matrix() *
        robot->getMass();

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

    robot_status.Update(time, q, v,
                        VectorX<double>::Zero(robot->get_num_actuators()),
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
      v, VectorX<double>::Zero(robot->get_num_velocities()), 1e-4,
      drake::MatrixCompareType::absolute));
}

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
