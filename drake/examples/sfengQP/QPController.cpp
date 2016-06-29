#include "drake/systems/plants/RigidBodyTree.h"
#include "drake/systems/plants/KinematicsCache.h"

#include "sfUtils.h"
#include "HumanoidState.h"
#include "QPController.h"

int QPController::control(
    const HumanoidState &rs,
    const QPInput &input,
    QPOutput &output) {
  if (!input.isSane()) {
    return -1;
  }
  ////////////////////////////////////////////////////////////////////
  // Inverse dynamics looks like:
  // M(q) * qdd + h(q,qd) = S * tau + J^T * lambda
  // M(q) is the intertia matrix, h(q, qd) is the gravitational and centrifugal
  // force, qdd is acceleration, S is the selection matrix (top 6 rows are
  // zeros due to the floating base), tau is joint torque, J^T is the transpose
  // of all contact Jacobian, and lambda is the contact wrench in the world
  // frame.
  // Note that since S.topRows(6) is zero,
  // tau = M_l * qdd + h_l - J^T_l * lamda, _l means the lower nTrq rows of
  // those matrices.
  // So we just need to solve for qdd and lambda, and tau can be computed as
  // above.
  //
  // We are assuming two foot contacts in this example.
  //
  // For the QP problem:
  // the unknown is _X = [qdd, lambda]
  // equality constraints:
  //  M_u * qdd + h_u = J^T_u * lambda (equations of motion)
  //  J * qdd + Jd * v = 0, (contact constraints)
  // inEquality: a bunch, joint torque limit, limits on lambda, etc
  // cost func:
  //  min (Jcom*qdd + Jcomd*v - comdd_d)^2
  //    + (qdd - qdd_d)^2
  //    + (lambda - lambda_d)^2
  //    + all_kinds_of_body_acceleration_cost_terms
  int nContacts = 2;
  int nQdd = rs.robot->number_of_velocities();
  int nWrench = 6 * nContacts;
  int nTrq = nQdd - 6;

  _nVar = nQdd + nWrench;
  _nEq = 6 + 6 * nContacts;
  _nInEq = 11 * nContacts + nTrq;
  this->resize();
  this->setZero();

  int qdd_start = 0;
  int lambda_start = nQdd;

  // tau = M_l * qdd + h_l - J^T_l * lambda,
  // tau = TAU * _X + tau0
  MatrixXd Tau(MatrixXd::Zero(nTrq, _nVar));
  Tau.block(0, qdd_start, nTrq, nQdd) = rs.M.bottomRows(nTrq);
  for (int i = 0; i < nContacts; i++) {
    Tau.block(0, lambda_start+i*6, nTrq, 6) =
      -rs.foot[i]->J.block(0, 6, 6, nTrq).transpose();
  }
  VectorXd tau0 = rs.h.tail(nTrq);


  ////////////////////////////////////////////////////////////////////
  // set up equality constraints
  // equations of motion part, 6 rows
  int rowIdx = 0;
  _CE.block(rowIdx, qdd_start, 6, nQdd) = rs.M.topRows(6);
  for (int i = 0; i < nContacts; i++) {
    _CE.block(rowIdx, lambda_start+i*6, 6, 6) =
      -rs.foot[i]->J.block<6, 6>(0, 0).transpose();
  }
  _ce0.segment<6>(rowIdx) = rs.h.head(6);
  rowIdx += 6;

  // contact constraints, 6 rows per contact
  for (int i = 0; i < nContacts; i++) {
    _CE.block(rowIdx, qdd_start, 6, nQdd) = rs.foot[i]->J;
    _ce0.segment<6>(rowIdx) = rs.foot[i]->Jdv - input.footdd_d[i];
    rowIdx += 6;
  }

  // i locked the qdd = zero here. In general, this is not true;
  // _CE.block(rowIdx,0,nQdd,nQdd).setIdentity();

  ////////////////////////////////////////////////////////////////////
  // set up inequality constraints
  // these are for contact wrench, 11 rows per contact.
  // NOTE: these constraints are specified for the contact wrench in the body
  // frame, but lambda are in world frame. So need to transform it by R^-1
  // 2 for Fx, Fy, Mz within friction cone, 6
  // 2 for CoP x within foot, 2
  // 2 for CoP y within foot, 2
  // 1 for Fz >= 0,
  rowIdx = 0;
  // Fz >= 0;
  for (int i = 0; i < nContacts; i++) {
    _CI(rowIdx, lambda_start+i*6 + 5) = 1;
    _ci_l(rowIdx) = 0;
    rowIdx++;
  }
  // Fx <= mu * Fz, Fx - mu * Fz <= 0
  for (int i = 0; i < nContacts; i++) {
    _CI(rowIdx, lambda_start+i*6 + 3) = 1;
    _CI(rowIdx, lambda_start+i*6 + 5) = -param.mu;
    _ci_u(rowIdx) = 0;
    rowIdx++;
  }
  // Fx >= -mu * Fz, Fx + mu * Fz >= 0
  for (int i = 0; i < nContacts; i++) {
    _CI(rowIdx, lambda_start+i*6 + 3) = 1;
    _CI(rowIdx, lambda_start+i*6 + 5) = param.mu;
    _ci_l(rowIdx) = 0;
    rowIdx++;
  }
  // Fy <= mu * Fz, Fy - mu * Fz <= 0
  for (int i = 0; i < nContacts; i++) {
    _CI(rowIdx, lambda_start+i*6 + 4) = 1;
    _CI(rowIdx, lambda_start+i*6 + 5) = -param.mu;
    _ci_u(rowIdx) = 0;
    rowIdx++;
  }
  // Fy >= -mu * Fz, Fy + mu * Fz >= 0
  for (int i = 0; i < nContacts; i++) {
    _CI(rowIdx, lambda_start+i*6 + 4) = 1;
    _CI(rowIdx, lambda_start+i*6 + 5) = param.mu;
    _ci_l(rowIdx) = 0;
    rowIdx++;
  }
  // Mz <= mu * Fz, Mz - mu * Fz <= 0
  for (int i = 0; i < nContacts; i++) {
    _CI(rowIdx, lambda_start+i*6 + 2) = 1;
    _CI(rowIdx, lambda_start+i*6 + 5) = -param.mu_Mz;
    _ci_u(rowIdx) = 0;
    rowIdx++;
  }
  // Mz >= -mu * Fz, Mz + mu * Fz >= 0
  for (int i = 0; i < nContacts; i++) {
    _CI(rowIdx, lambda_start+i*6 + 2) = 1;
    _CI(rowIdx, lambda_start+i*6 + 5) = param.mu_Mz;
    _ci_l(rowIdx) = 0;
    rowIdx++;
  }
  // cop_x <= x_max, -My / Fz <= x_max, -My - x_max * Fz <= 0
  for (int i = 0; i < nContacts; i++) {
    _CI(rowIdx, lambda_start+i*6 + 1) = -1;
    _CI(rowIdx, lambda_start+i*6 + 5) = -param.x_max;
    _ci_u(rowIdx) = 0;
    rowIdx++;
  }
  // cop_x >= x_min, -My / Fz >= x_min, -My - x_min * Fz >= 0
  for (int i = 0; i < nContacts; i++) {
    _CI(rowIdx, lambda_start+i*6 + 1) = -1;
    _CI(rowIdx, lambda_start+i*6 + 5) = -param.x_min;
    _ci_l(rowIdx) = 0;
    rowIdx++;
  }
  // cop_y <= y_max, Mx / Fz <= y_max, Mx - y_max * Fz <= 0
  for (int i = 0; i < nContacts; i++) {
    _CI(rowIdx, lambda_start+i*6 + 0) = 1;
    _CI(rowIdx, lambda_start+i*6 + 5) = -param.y_max;
    _ci_u(rowIdx) = 0;
    rowIdx++;
  }
  // cop_y >= y_min, Mx / Fz >= y_min, Mx - y_min * Fz >= 0
  for (int i = 0; i < nContacts; i++) {
    _CI(rowIdx, lambda_start+i*6 + 0) = 1;
    _CI(rowIdx, lambda_start+i*6 + 5) = -param.y_min;
    _ci_l(rowIdx) = 0;
    rowIdx++;
  }
  // Since all of the above are constraints on wrench expressed in the body
  // frame, we need to rotate the lambda into body frame.
  MatrixXd world_to_foot(MatrixXd::Zero(nWrench, nWrench));
  for (int i = 0; i < nContacts; i++) {
    world_to_foot.block<3, 3>(i*6, i*6) = rs.foot[i]->pose.linear().transpose();
    world_to_foot.block<3, 3>(i*6+3, i*6+3) =
      world_to_foot.block<3, 3>(i*6, i*6);
  }
  _CI.block(0, lambda_start, rowIdx, nWrench) =
    _CI.block(0, lambda_start, rowIdx, nWrench) * world_to_foot;

  // torque limits: min <= tau <= max, nTrq rows
  // min <= M_l * qdd + h_l - J^T_l * lambda <= max
  // min - h_l <= M_l * qdd - J^T_l * lambda <= max - h_l
  // tau = rs.robot->B.bottomRows(nTrq) * u,
  // u = rs.robot->B.bottomRows(nTrq).transpose() * tau
  // since B should be orthonormal.
  // tau is joint space indexed, and u is actuator space indexed.
  // constraints are specified with u index.
  _CI.block(rowIdx, 0, nTrq, _nVar) =
    rs.robot->B.bottomRows(nTrq).transpose() * Tau;
  _ci_l.segment(rowIdx, nTrq) =
    -rs.robot->B.bottomRows(nTrq).transpose() * tau0;
  _ci_u.segment(rowIdx, nTrq) =
    -rs.robot->B.bottomRows(nTrq).transpose() * tau0;
  for (size_t i = 0; i < rs.robot->actuators.size(); i++) {
    _ci_l(rowIdx+i) += rs.robot->actuators[i].effort_limit_min;
    _ci_u(rowIdx+i) += rs.robot->actuators[i].effort_limit_max;
  }
  rowIdx += nTrq;

  ////////////////////////////////////////////////////////////////////
  // cost function:
  // min 0.5 * _X^T * _H * _X + _h0.transpose() * _X, and we are sending _H, _h0
  // CoM term
  // w * (J * qdd + Jdv - comdd_d)^T * (J * qdd + Jdv - comdd_d)
  _H.block(qdd_start, qdd_start, nQdd, nQdd) +=
    input.w_com * rs.J_com.transpose() * rs.J_com;
  _h0.segment(qdd_start, nQdd) +=
    input.w_com * (rs.Jdv_com - input.comdd_d).transpose() * rs.J_com;

  // pelvis, same as above, you can add torso, hand, head, etc
  _H.block(qdd_start, qdd_start, nQdd, nQdd) +=
    input.w_pelv * rs.pelv.J.transpose() * rs.pelv.J;
  _h0.segment(qdd_start, nQdd) +=
    input.w_pelv * (rs.pelv.Jdv - input.pelvdd_d).transpose() * rs.pelv.J;

  // regularize qdd to qdd_d
  _H.block(qdd_start, qdd_start, nQdd, nQdd) +=
    input.w_qdd * MatrixXd::Identity(nQdd, nQdd);
  _h0.segment(qdd_start, nQdd) += input.w_qdd * (-input.qdd_d);

  // regularize lambda to lambda_d
  VectorXd lambda_d(VectorXd::Zero(nWrench));
  lambda_d[5] = 660;
  lambda_d[11] = 660;
  _H.block(lambda_start, lambda_start, nWrench, nWrench) +=
    input.w_wrench_reg * MatrixXd::Identity(nWrench, nWrench);
  _h0.segment(lambda_start, nWrench) += input.w_wrench_reg * (-lambda_d);

  ////////////////////////////////////////////////////////////////////
  // SOLVE
  this->solve();

  ////////////////////////////////////////////////////////////////////
  // parse output,
  // compute qdd, lambda, tau
  output.qdd = _X.segment(qdd_start, nQdd);
  output.trq = tau0 + Tau * _X;

  output.comdd = rs.J_com * output.qdd + rs.Jdv_com;
  output.pelvdd = rs.pelv.J * output.qdd + rs.pelv.Jdv;
  output.torsodd = rs.torso.J * output.qdd + rs.torso.Jdv;

  // transform lambda into the foot force torque sensor frame, so we can
  // directly compare them with the measured values.
  for (int i = 0; i < nContacts; i++) {
    output.footdd[i] = (rs.foot[i]->J * output.qdd + rs.foot[i]->Jdv);
    output.foot_wrench_w[i] = _X.segment<6>(lambda_start+i*6);

    Isometry3d T(Isometry3d::Identity());
    T.translation() = rs.foot[i]->pose.translation()
                    - rs.foot_sensor[i]->pose.translation();

    output.foot_wrench_in_sensor_frame[i] =
      transformSpatialForce(T, output.foot_wrench_w[i]);

    output.foot_wrench_in_sensor_frame[i].head<3>() =
      rs.foot_sensor[i]->pose.linear().transpose() *
      output.foot_wrench_in_sensor_frame[i].head<3>();
    output.foot_wrench_in_sensor_frame[i].tail<3>() =
      rs.foot_sensor[i]->pose.linear().transpose() *
      output.foot_wrench_in_sensor_frame[i].tail<3>();
  }

  ////////////////////////////////////////////////////////////////////
  // sanity checks,
  // check dynamics
  VectorXd residual = rs.M * output.qdd + rs.h;
  for (int i = 0; i < nContacts; i++)
    residual -= rs.foot[i]->J.transpose() * output.foot_wrench_w[i];
  residual.tail(nTrq) -= output.trq;

  if (!residual.isZero()) {
    return -1;
  }

  for (int i = 0; i < nContacts; i++) {
    if (!output.footdd[i].isZero()) {
      return -1;
    }
  }

  if (!output.isSane()) {
    return -1;
  }

  return 0;
}


void QPOutput::print() const {
  std::cout << "===============================================\n";
  std::cout << "accelerations:\n";
  for (int i = 0; i < qdd.rows(); i++)
    std::cout << jointNames[i] << ": " << qdd[i] << std::endl;

  std::cout << "com acc: ";
  std::cout << comdd.transpose() << std::endl;

  std::cout << "pelv acc: ";
  std::cout << pelvdd.transpose() << std::endl;

  std::cout << "torso acc: ";
  std::cout << torsodd.transpose() << std::endl;

  std::cout << "left foot acc: " <<
    footdd[Side::LEFT].transpose() << std::endl;
  std::cout << "right foot acc: " <<
    footdd[Side::RIGHT].transpose() << std::endl;

  std::cout << "===============================================\n";
  std::cout << "left foot wrench_w: " <<
    foot_wrench_w[Side::LEFT].transpose() << std::endl;
  std::cout << "right foot wrench_w: " <<
    foot_wrench_w[Side::RIGHT].transpose() << std::endl;
  std::cout << "left foot wrench in sensor frame: " <<
    foot_wrench_in_sensor_frame[Side::LEFT].transpose() << std::endl;
  std::cout << "right foot wrench in sensor frame: " <<
    foot_wrench_in_sensor_frame[Side::RIGHT].transpose() << std::endl;

  std::cout << "===============================================\n";
  std::cout << "torque:\n";
  for (int i = 0; i < trq.rows(); i++)
    std::cout << jointNames[i+6] << ": " << trq[i] << std::endl;
}

