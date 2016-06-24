#include "QPController.h"
#include "QPEstimator.h"

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
  
  // make controller
  QPController con;
  QPInput input(*rs.robot);
  QPOutput output(*rs.robot);

  // setup input
  assert(input.loadParamFromFile(std::string(VALKYRIE_URDF_PATH) + std::string("/qpc_params")));

  input.comdd_d.setZero();
  input.pelvdd_d.setZero();
  input.torsodd_d.setZero();
  input.footdd_d[Side::LEFT].setZero();
  input.footdd_d[Side::RIGHT].setZero();
  input.qdd_d.setZero();

  // call QP
  con.control(rs, input, output);

  // print result
  output.print();

  // make estimator
  QPEstimator est(urdf);
  assert(est.loadParamFromFile(std::string(VALKYRIE_URDF_PATH) + std::string("/qpe_params")));

  est.init(0, q, qd, output.trq, output.foot_wrench_in_sensor_frame[0], output.foot_wrench_in_sensor_frame[1]);
  est.estimate(0, q, qd + output.qdd * est.dt, output.trq, output.foot_wrench_in_sensor_frame[0], output.foot_wrench_in_sensor_frame[1]);

  std::cout << "residual:\n" << est.residual << std::endl;
  std::cout << "\nvel:\n" << est.vel << std::endl;
  std::cout << "\ntrq:\n" << est.trq << std::endl;
  std::cout << "\nwrench:\n" << est.wrench << std::endl;
  /*
  */

  return 1;
}
