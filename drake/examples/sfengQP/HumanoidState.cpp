#include "HumanoidState.h"
#include <iostream>

void HumanoidState::_fillKinematics(
    const std::string &name,
    Isometry3d &pose,
    Vector6d &vel,
    MatrixXd &J,
    Vector6d &Jdv,
    const Vector3d &local_offset) {

  int id = bodyName2ID.at(name);
  pose = Isometry3d::Identity();
  pose.translation() = local_offset;
  pose = robot->relativeTransform(cache, 0, id) * pose;

  vel = getTaskSpaceVel(*(robot), cache, id, local_offset);
  J = getTaskSpaceJacobian(*(robot), cache, id, local_offset);
  Jdv = getTaskSpaceJacobianDotTimesV(*(robot), cache, id, local_offset);
}

void HumanoidState::update(double t,
    const VectorXd &q,
    const VectorXd &v,
    const VectorXd &trq,
    const Vector6d &l_ft,
    const Vector6d &r_ft,
    const Matrix3d &rot) {
  if (q.size() != this->pos.size() ||
      v.size() != this->vel.size() ||
      trq.size() != this->trq.size()) {
    throw std::runtime_error("robot state update dimension mismatch");
  }

  time = t;
  this->pos = q;
  this->vel = v;
  this->trq = trq;

  cache.initialize(pos, vel);
  robot->doKinematics(cache, true);

  M = robot->massMatrix(cache);
  eigen_aligned_unordered_map<RigidBody const*,
    Matrix<double, TWIST_SIZE, 1>> f_ext;
  h = robot->dynamicsBiasTerm(cache, f_ext);

  // com
  com = robot->centerOfMass(cache);
  J_com = robot->centerOfMassJacobian(cache);
  Jdv_com = robot->centerOfMassJacobianDotTimesV(cache);
  comd = J_com * v;

  // body parts
  _fillKinematics(pelv.link_name, pelv.pose, pelv.vel, pelv.J, pelv.Jdv);
  // the fictionary contact point is 9cm below the ankle joint
  _fillKinematics(l_foot.link_name, l_foot.pose, l_foot.vel,
      l_foot.J, l_foot.Jdv, Vector3d(0, 0, -0.09));
  _fillKinematics(r_foot.link_name, r_foot.pose, r_foot.vel,
      r_foot.J, r_foot.Jdv, Vector3d(0, 0, -0.09));
  _fillKinematics(torso.link_name, torso.pose, torso.vel, torso.J, torso.Jdv);

  _fillKinematics(l_foot_sensor.link_name, l_foot_sensor.pose,
      l_foot_sensor.vel, l_foot_sensor.J, l_foot_sensor.Jdv,
      Vector3d(0.0215646, 0.0, -0.051054));
  _fillKinematics(r_foot_sensor.link_name, r_foot_sensor.pose,
      r_foot_sensor.vel, r_foot_sensor.J, r_foot_sensor.Jdv,
      Vector3d(0.0215646, 0.0, -0.051054));

  // ft sensor
  footFT_b[Side::LEFT] = l_ft;
  footFT_b[Side::RIGHT] = r_ft;
  for (int i = 0; i < 2; i++) {
    footFT_b[i].head(3) = rot * footFT_b[i].head(3);
    footFT_b[i].tail(3) = rot * footFT_b[i].tail(3);
    footFT_w[i].head(3) = foot_sensor[i]->pose.linear() * footFT_b[i].head(3);
    footFT_w[i].tail(3) = foot_sensor[i]->pose.linear() * footFT_b[i].tail(3);
  }

  // cop
  Vector2d cop_w[2];
  double Fz[2];
  for (int i = 0; i < 2; i++) {
    Fz[i] = footFT_w[i][5];
    // cop relative to the ft sensor
    cop_b[i][0] = -footFT_b[i][1] / footFT_b[i][5];
    cop_b[i][1] = footFT_b[i][0] / footFT_b[i][5];

    cop_w[i][0] = -footFT_w[i][1] / Fz[i]
      + foot_sensor[i]->pose.translation()[0];
    cop_w[i][1] = footFT_w[i][0] / Fz[i]
      + foot_sensor[i]->pose.translation()[1];
  }

  cop = (cop_w[Side::LEFT]*Fz[Side::LEFT] + cop_w[Side::RIGHT]*Fz[Side::RIGHT])
      / (Fz[Side::LEFT] + Fz[Side::RIGHT]);
}
