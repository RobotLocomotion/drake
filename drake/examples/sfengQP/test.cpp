#include "QPController.h"

int main()
{
  ////////////////////////////////////////////////////////////////////
  // load model
  std::string urdf = std::string(VALKYRIE_URDF_PATH) + std::string("/valkyrie_sim_drake.urdf");
  sfRobotState rs(std::unique_ptr<RigidBodyTree>(new RigidBodyTree(urdf, DrakeJoint::ROLLPITCHYAW)));

  ////////////////////////////////////////////////////////////////////
  // set state and do kinematics
  VectorXd q(rs.robot->number_of_positions());
  VectorXd qd(rs.robot->number_of_velocities());

  q.setZero();
  qd.setZero();

  q[rs.jointName2ID.at("rightHipRoll")] = 0.01;
  q[rs.jointName2ID.at("rightHipPitch")] = -0.5432;
  q[rs.jointName2ID.at("rightKneePitch")] = 1.2195;
  q[rs.jointName2ID.at("rightAnklePitch")] = -0.7070;
  q[rs.jointName2ID.at("rightAnkleRoll")] = -0.0069;

  q[rs.jointName2ID.at("leftHipRoll")] = -0.01;
  q[rs.jointName2ID.at("leftHipPitch")] = -0.5432;
  q[rs.jointName2ID.at("leftKneePitch")] = 1.2195;
  q[rs.jointName2ID.at("leftAnklePitch")] = -0.7070;
  q[rs.jointName2ID.at("leftAnkleRoll")] = 0.0069;

  q[rs.jointName2ID.at("rightShoulderRoll")] = 1;
  q[rs.jointName2ID.at("rightShoulderYaw")] = 0.5;
  q[rs.jointName2ID.at("rightElbowPitch")] = M_PI/2.;
  
  q[rs.jointName2ID.at("leftShoulderRoll")] = -1;
  q[rs.jointName2ID.at("leftShoulderYaw")] = 0.5;
  q[rs.jointName2ID.at("leftElbowPitch")] = -M_PI/2.;
   
  rs.update(0, q, qd, VectorXd::Zero(rs.robot->number_of_velocities()-6), Vector6d::Zero(), Vector6d::Zero());
  
  QPController con;
  QPInput input(*rs.robot);
  QPOutput output(*rs.robot);

  con.control(rs, input, output);
  output.print();

  return 1;
}
