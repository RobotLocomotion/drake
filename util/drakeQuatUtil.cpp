#include "drakeQuatUtil.h"
#include <iostream>

using namespace Eigen;

Eigen::Vector4d quatConjugate(const Eigen::Vector4d& q)
{
  Vector4d q_conj;
  q_conj << q(0), -q(1), -q(2), -q(3);
  return q_conj;
}

Eigen::Matrix4d dquatConjugate(const Eigen::Vector4d& q)
{
  Matrix4d dq_conj = Matrix4d::Identity();
  dq_conj(1, 1) = -1.0;
  dq_conj(2, 2) = -1.0;
  dq_conj(3, 3) = -1.0;
  return dq_conj;
}

Eigen::Vector4d quatProduct(const Eigen::Vector4d& q1, const Eigen::Vector4d& q2)
{
  double w1 = q1(0);
  double w2 = q2(0);
  const auto& v1 = q1.tail<3>();
  const auto& v2 = q2.tail<3>();
  Vector4d r;
  r << w1 * w2 - v1.dot(v2), v1.cross(v2) + w1 * v2 + w2 * v1;
  return r;
}

Eigen::Matrix<double, 4, 8> dquatProduct(const Eigen::Vector4d& q1, const Eigen::Vector4d& q2)
{
  double w1 = q1(0);
  double w2 = q2(0);
  const auto& v1 = q1.tail<3>();
  const auto& v2 = q2.tail<3>();

  Matrix<double, 4, 8> dr;
  dr.row(0) << w2, -v2.transpose(), w1, -v1.transpose();
  dr.row(1) << q2(1), q2(0), q2(3), -q2(2), q1(1), q1(0), -q1(3), q1(2);
  dr.row(2) << q2(2), -q2(3), q2(0), q2(1), q1(2), q1(3), q1(0), -q1(1);
  dr.row(3) << q2(3), q2(2), -q2(1), q2(0), q1(3), -q1(2), q1(1), q1(0);
  return dr;
}

Eigen::Vector3d quatRotateVec(const Eigen::Vector4d& q, const Eigen::Vector3d& v)
{
  Vector4d v_quat;
  v_quat << 0, v;
  Vector4d q_times_v = quatProduct(q, v_quat);
  Vector4d q_conj = quatConjugate(q);
  Vector4d v_rot = quatProduct(q_times_v, q_conj);
  Vector3d r = v_rot.bottomRows<3>();
  return r;
}

Eigen::Matrix<double, 3, 7> dquatRotateVec(const Eigen::Vector4d& q, const Eigen::Vector3d& v)
{
  Matrix<double, 4, 7> dq;
  dq << Matrix4d::Identity(), MatrixXd::Zero(4, 3);
  Matrix<double, 4, 7> dv = Matrix<double, 4, 7>::Zero();
  dv.bottomRightCorner<3, 3>() = Matrix3d::Identity();
  Matrix<double, 8, 7> dqdv;
  dqdv << dq, dv;

  Vector4d v_quat;
  v_quat << 0, v;
  Vector4d q_times_v = quatProduct(q, v_quat);
  Matrix<double, 4, 8> dq_times_v_tmp = dquatProduct(q, v_quat);
  Matrix<double, 4, 7> dq_times_v = dq_times_v_tmp * dqdv;

  Matrix<double, 4, 7> dq_conj = dquatConjugate(q) * dq;
  Matrix<double, 8, 7> dq_times_v_dq_conj;
  dq_times_v_dq_conj << dq_times_v, dq_conj;
  Matrix<double, 4, 8> dv_rot_tmp = dquatProduct(q_times_v, quatConjugate(q));
  Matrix<double, 4, 7> dv_rot = dv_rot_tmp * dq_times_v_dq_conj;
  Eigen::Matrix<double, 3, 7> dr = dv_rot.bottomRows(3);
  return dr;
}

Eigen::Vector4d quatDiff(const Eigen::Vector4d& q1, const Eigen::Vector4d& q2)
{
  return quatProduct(quatConjugate(q1), q2);
}

Eigen::Matrix<double, 4, 8> dquatDiff(const Eigen::Vector4d& q1, const Eigen::Vector4d& q2)
{
  auto dr = dquatProduct(quatConjugate(q1), q2);
  dr.block<4, 3>(0, 1) = -dr.block<4, 3>(0, 1);
  return dr;
}

double quatDiffAxisInvar(const Eigen::Vector4d& q1, const Eigen::Vector4d& q2, const Eigen::Vector3d& u)
{
  Vector4d r = quatDiff(q1, q2);
  double e = -2.0 + 2 * r(0) * r(0) + 2 * pow(u(0) * r(1) + u(1) * r(2) + u(2) * r(3), 2);
  return e;
}

Eigen::Matrix<double, 1, 11> dquatDiffAxisInvar(const Eigen::Vector4d& q1, const Eigen::Vector4d& q2, const Eigen::Vector3d& u)
{
  Vector4d r = quatDiff(q1, q2);
  Matrix<double, 4, 8> dr = dquatDiff(q1, q2);
  Matrix<double, 1, 11> de;
  const auto& rvec = r.tail<3>();
  de << 4 * r(0) * dr.row(0) + 4 * u.transpose() * rvec *u.transpose() * dr.block<3, 8>(1, 0), 4 * u.transpose() * rvec * rvec.transpose();
  return de;
}
