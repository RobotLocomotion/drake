#include "drake/Path.h"
#include "qp_controller.h"

QPOutput TestGravityCompensation(const HumanoidStatus& rs) {
  // Make controller.
  QPController con;
  QPInput input;
  QPOutput output;
  InitQPOutput(rs.robot(), output);
  InitQPInput(rs.robot(), input);

  // Setup QP controller's parameter.
  con.param.mu = 1;
  con.param.mu_Mz = 0.1;
  con.param.x_max = 0.2;
  con.param.x_min = -0.05;
  con.param.y_max = 0.05;
  con.param.y_min = -0.05;

  // Make input.
  input.comdd_d.setZero();
  input.pelvdd_d.setZero();
  input.torsodd_d.setZero();
  input.footdd_d[Side::LEFT].setZero();
  input.footdd_d[Side::RIGHT].setZero();
  input.wrench_d[Side::LEFT].setZero();
  input.wrench_d[Side::RIGHT].setZero();
  input.vd_d.setZero();

  // [5] is Fz, 660N * 2 is about robot weight.
  input.wrench_d[Side::LEFT][5] = 660;
  input.wrench_d[Side::RIGHT][5] = 660;

  input.w_com = 1e2;
  input.w_pelv = 1e1;
  input.w_torso = 1e1;
  input.w_foot = 1e1;
  input.w_vd = 1e3;
  input.w_wrench_reg = 1e-5;

  ////////////////////////////////////////////////////////////////////
  // Call QP.
  con.Control(rs, input, output);

  // Print results.
  PrintQPOutput(output);

  // Print quadratic costs for all the terms.
  ComputeQPCost(rs, input, output);

  return output;
}

int main() {
  ////////////////////////////////////////////////////////////////////
  // Load model.
  std::string urdf =
      Drake::getDrakePath() +
      std::string(
          "/examples/QPInverseDynamicsForHumanoids/valkyrie_sim_drake.urdf");
  HumanoidStatus rs(std::unique_ptr<RigidBodyTree>(
      new RigidBodyTree(urdf, DrakeJoint::ROLLPITCHYAW)));

  ////////////////////////////////////////////////////////////////////
  // Set state and do kinematics.
  VectorXd q(rs.robot().number_of_positions());
  VectorXd qd(rs.robot().number_of_velocities());

  q.setZero();
  qd.setZero();

  // These corresponds to a nominal pose for the Valkyrie robot: slightly
  // crouched, arm raised a bit.
  q[rs.joint_name_to_id().at("rightHipRoll")] = 0.01;
  q[rs.joint_name_to_id().at("rightHipPitch")] = -0.5432;
  q[rs.joint_name_to_id().at("rightKneePitch")] = 1.2195;
  q[rs.joint_name_to_id().at("rightAnklePitch")] = -0.7070;
  q[rs.joint_name_to_id().at("rightAnkleRoll")] = -0.0069;

  q[rs.joint_name_to_id().at("leftHipRoll")] = -0.01;
  q[rs.joint_name_to_id().at("leftHipPitch")] = -0.5432;
  q[rs.joint_name_to_id().at("leftKneePitch")] = 1.2195;
  q[rs.joint_name_to_id().at("leftAnklePitch")] = -0.7070;
  q[rs.joint_name_to_id().at("leftAnkleRoll")] = 0.0069;

  q[rs.joint_name_to_id().at("rightShoulderRoll")] = 1;
  q[rs.joint_name_to_id().at("rightShoulderYaw")] = 0.5;
  q[rs.joint_name_to_id().at("rightElbowPitch")] = M_PI / 2.;

  q[rs.joint_name_to_id().at("leftShoulderRoll")] = -1;
  q[rs.joint_name_to_id().at("leftShoulderYaw")] = 0.5;
  q[rs.joint_name_to_id().at("leftElbowPitch")] = -M_PI / 2.;

  rs.Update(0, q, qd, VectorXd::Zero(rs.robot().actuators.size()),
            Vector6d::Zero(), Vector6d::Zero());

  QPOutput output = TestGravityCompensation(rs);

  return 0;
}
