#include "QPEstimator.h"

int QPEstimator::estimate(double t, const VectorXd &q, const VectorXd &v, const VectorXd &trq, const Vector6d &ft_l_b, const Vector6d &ft_r_b) 
{
  //rs.update(t, q, v, trq, ft_l_b, ft_r_b);

  int nContacts = 2;
  int nQdd = rs.robot->number_of_velocities();
  int nWrench = 6 * nContacts;
  int nTrq = nQdd - 6;
  
  // solve for qd_{k+1}, error, wrench 
  _nVar = nQdd + nQdd + nWrench;
  _nEq = 6 + 6 * nContacts;
  _nInEq = 11 * nContacts + nTrq;
  this->resize();
  this->setZero();

  int qd_nxt_start = 0;
  int error_start = nQdd;
  int lambda_start = 2*nQdd;
  
  // EOM:
  // M * qdd + h = S * tau + J^T * lambda + error 
  // error = M_l * qdd + h_l - J^T * lambda - S * tau
  
  // tau = M_l * (qd_nxt - qd) / dt - error_l + h_l - J^T_l * lambda,
  // tau = TAU * _X + tau0
  MatrixXd Tau(MatrixXd::Zero(nTrq, _nVar));
  VectorXd tau0 = rs.h.tail(nTrq); 
  Tau.block(0,qd_nxt_start,nTrq,nQdd) = rs.M.bottomRows(nTrq) / dt;
  tau0 -= rs.M.bottomRows(nTrq) * rs.vel / dt;
  Tau.block(0,error_start+6,nTrq,nTrq) = -MatrixXd::Identity(nTrq, nTrq);
  for (int i = 0; i < nContacts; i++) {
    Tau.block(0,lambda_start+i*6,nTrq,6) = -rs.foot_sensor[i]->J.block(0,6,6,nTrq).transpose();
  }
  
  ////////////////////////////////////////////////////////////////////
  // set up equality constraints
  // equations of motion part, 6 rows
  // M_u * (qd_nxt - qd) / dt + h_u - J^T_u * lambda - error_u = 0
  int rowIdx = 0;
  _CE.block(rowIdx,qd_nxt_start,6,nQdd) = rs.M.topRows(6) / dt;
  _CE.block(rowIdx,error_start,6,6) = -Matrix<double,6,6>::Identity();
  for (int i = 0; i < nContacts; i++) {
    _CE.block(rowIdx,lambda_start+i*6,6,6) = -rs.foot_sensor[i]->J.block<6,6>(0,0).transpose();
  }
  _ce0.segment<6>(rowIdx) = rs.h.head(6);
  _ce0.segment<6>(rowIdx) -= rs.M.topRows(6) / dt * rs.vel;
  rowIdx += 6;

  // contact constraints, 6 rows per contact
  // J * (qd_nxt - qd) / dt + Jdv = 0;
  for (int i = 0; i < nContacts; i++) {
    _CE.block(rowIdx,qd_nxt_start,6,nQdd) = rs.foot_sensor[i]->J / dt;
    _ce0.segment<6>(rowIdx) = rs.foot_sensor[i]->Jdv - rs.foot_sensor[i]->J * rs.vel / dt;
    rowIdx += 6;
  }
  
  ////////////////////////////////////////////////////////////////////
  // set up inequality constraints
  // these are for contact wrench, 11 rows per contact.
  // NOTE: these constraints are specified for the contact wrench in the body
  // frame, but lambda are in world frame. So need to transform it by R^-1 
  // 2 for Fx, Fy, Mz within friction cone, 6
  // 2 for CoP x within foot, 2
  // 2 for CoP y within foot, 2
  // 1 for Fz >= 0,
  double mu = 1;
  double x_max = 0.2;
  double x_min = -0.05;
  double y_max = 0.05;
  double y_min = -0.05;
  
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
    _CI(rowIdx, lambda_start+i*6 + 5) = -mu;
    _ci_u(rowIdx) = 0;
    rowIdx++;
  }
  // Fx >= -mu * Fz, Fx + mu * Fz >= 0
  for (int i = 0; i < nContacts; i++) {
    _CI(rowIdx, lambda_start+i*6 + 3) = 1;
    _CI(rowIdx, lambda_start+i*6 + 5) = mu;
    _ci_l(rowIdx) = 0;
    rowIdx++;
  }
  // Fy <= mu * Fz, Fy - mu * Fz <= 0
  for (int i = 0; i < nContacts; i++) {
    _CI(rowIdx, lambda_start+i*6 + 4) = 1;
    _CI(rowIdx, lambda_start+i*6 + 5) = -mu;
    _ci_u(rowIdx) = 0;
    rowIdx++;
  }
  // Fy >= -mu * Fz, Fy + mu * Fz >= 0
  for (int i = 0; i < nContacts; i++) {
    _CI(rowIdx, lambda_start+i*6 + 4) = 1;
    _CI(rowIdx, lambda_start+i*6 + 5) = mu;
    _ci_l(rowIdx) = 0;
    rowIdx++;
  }
  // Mz <= mu * Fz, Mz - mu * Fz <= 0
  for (int i = 0; i < nContacts; i++) {
    _CI(rowIdx, lambda_start+i*6 + 2) = 1;
    _CI(rowIdx, lambda_start+i*6 + 5) = -mu;
    _ci_u(rowIdx) = 0;
    rowIdx++;
  }
  // Mz >= -mu * Fz, Mz + mu * Fz >= 0
  for (int i = 0; i < nContacts; i++) {
    _CI(rowIdx, lambda_start+i*6 + 2) = 1;
    _CI(rowIdx, lambda_start+i*6 + 5) = mu;
    _ci_l(rowIdx) = 0;
    rowIdx++;
  }
  // cop_x <= x_max, -My / Fz <= x_max, -My - x_max * Fz <= 0
  for (int i = 0; i < nContacts; i++) {
    _CI(rowIdx, lambda_start+i*6 + 1) = -1;
    _CI(rowIdx, lambda_start+i*6 + 5) = -x_max;
    _ci_u(rowIdx) = 0;
    rowIdx++;
  }
  // cop_x >= x_min, -My / Fz >= x_min, -My - x_min * Fz >= 0
  for (int i = 0; i < nContacts; i++) {
    _CI(rowIdx, lambda_start+i*6 + 1) = -1;
    _CI(rowIdx, lambda_start+i*6 + 5) = -x_min;
    _ci_l(rowIdx) = 0;
    rowIdx++;
  }
  // cop_y <= y_max, Mx / Fz <= y_max, Mx - y_max * Fz <= 0
  for (int i = 0; i < nContacts; i++) {
    _CI(rowIdx, lambda_start+i*6 + 0) = 1;
    _CI(rowIdx, lambda_start+i*6 + 5) = -y_max;
    _ci_u(rowIdx) = 0;
    rowIdx++;
  }
  // cop_y >= y_min, Mx / Fz >= y_min, Mx - y_min * Fz >= 0
  for (int i = 0; i < nContacts; i++) {
    _CI(rowIdx, lambda_start+i*6 + 0) = 1;
    _CI(rowIdx, lambda_start+i*6 + 5) = -y_min;
    _ci_l(rowIdx) = 0;
    rowIdx++;
  }
  // since all of the above are constraints on wrench expressed in the body frame,
  // we need to rotate the lambda into body frame
  MatrixXd world_to_foot(MatrixXd::Zero(nWrench, nWrench));
  for (int i = 0; i < nContacts; i++) {
    world_to_foot.block<3,3>(i*6,i*6) = rs.foot_sensor[i]->pose.linear().transpose();
    world_to_foot.block<3,3>(i*6+3,i*6+3) = world_to_foot.block<3,3>(i*6,i*6);
  }
  _CI.block(0,lambda_start,rowIdx,nWrench) = _CI.block(0,lambda_start,rowIdx,nWrench) * world_to_foot;

  // torque limits: min <= tau <= max, nTrq rows
  // min <= Tau * X + tau0 <= max
  // min - h_l <= Tau * X + tau0 <= max - h_l
  VectorXd tau_min = VectorXd::Constant(nTrq, -300);
  VectorXd tau_max = VectorXd::Constant(nTrq, 300);
  _CI.block(rowIdx,0,nTrq,_nVar) = Tau; 
  _ci_l.segment(rowIdx,nTrq) = tau_min - tau0;
  _ci_u.segment(rowIdx,nTrq) = tau_max - tau0;
  rowIdx += nTrq;
  
  ////////////////////////////////////////////////////////////////////
  // cost function: 
  // min error^2 + measurement_error^2
  // error^2 term
  _H.block(error_start,error_start,nQdd,nQdd) += w_error * MatrixXd::Identity(nQdd,nQdd);
  _h0.segment(error_start,nQdd) += w_error * VectorXd::Zero(nQdd);

  // measurement errors:
  // vel term
  _H.block(qd_nxt_start,qd_nxt_start,nQdd,nQdd) += w_measured_vel * MatrixXd::Identity(nQdd,nQdd);
  _h0.segment(qd_nxt_start,nQdd) += w_measured_vel * (-v);

  // wrench term, stuff measured in the body frame
  _H.block(lambda_start,lambda_start,nWrench,nWrench) += w_measured_wrench * world_to_foot;
  _h0.segment<6>(lambda_start) += w_measured_wrench * (-ft_l_b);
  _h0.segment<6>(lambda_start+6) += w_measured_wrench * (-ft_r_b);

  // measured torque
  // (Tau * X + tau0 - trq)^T * (Tau * X + tau0 - trq)
  _H += w_measured_trq * Tau.transpose() * Tau;
  _h0 += w_measured_trq * (tau0 - trq).transpose() * Tau;

  ////////////////////////////////////////////////////////////////////
  // SOLVE
  /*
  _X.segment(qd_nxt_start, nQdd) = v;
  _X.segment(error_start, nQdd).setZero();
  _X.segment(lambda_start, 6) = ft_l_b;
  _X.segment(lambda_start+6, 6) = ft_r_b;
  _X.segment(lambda_start, nWrench) = world_to_foot.transpose() * _X.segment(lambda_start, nWrench); 

  std::cout << "qdd:\n" << (_X.segment(qd_nxt_start, nQdd) - rs.vel) / dt << std::endl;

  std::cout << "eq:\n" << (_CE * _X + _ce0) << std::endl;
  std::cout << "ineq_l:\n" << (_CI * _X - _ci_l) << std::endl;
  std::cout << "ineq_u:\n" << (_ci_u - _CI * _X) << std::endl;
  */

  this->solve();
  
  ////////////////////////////////////////////////////////////////////
  // parse output,
  // compute qdd, lambda, tau
  this->vel = _X.segment(qd_nxt_start, nQdd);
  this->residual = _X.segment(error_start, nQdd);
  this->trq = Tau * _X + tau0;
  this->wrench = _X.segment(lambda_start, nWrench);

  return 0;
}

int QPEstimator::init(double t, const VectorXd &q, const VectorXd &v, const VectorXd &trq, const Vector6d &ft_l, const Vector6d &ft_r)
{
  rs.update(t, q, v, trq, ft_l, ft_r);
  _inited = true;

  return 0;
}
