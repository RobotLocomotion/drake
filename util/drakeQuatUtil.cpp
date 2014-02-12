#include "drakeQuatUtil.h"
#include <iostream>

using namespace Eigen;
void quatConjugate(const Vector4d &q, Vector4d &q_conj)
{
  q_conj<<q(0),-q(1),-q(2),-q(3);
}

void quatConjugate(const Vector4d &q, Vector4d &q_conj, Matrix4d &dq_conj)
{
  quatConjugate(q,q_conj);
  dq_conj = Matrix4d::Identity();
  dq_conj(1,1) = -1.0;
  dq_conj(2,2) = -1.0;
  dq_conj(3,3) = -1.0;
}
void quatProduct(const Vector4d &q1,const Vector4d &q2, Vector4d &r)
{
  double w1 = q1(0);
  double w2 = q2(0);
  Vector3d v1,v2;
  v1 = q1.tail(3);
  v2 = q2.tail(3);
  r << w1*w2-v1.dot(v2), v1.cross(v2)+w1*v2+w2*v1;
}

void quatProduct(const Vector4d &q1, const Vector4d &q2, Vector4d &r, Matrix<double,4,8> &dr)
{
  double w1 = q1(0);
  double w2 = q2(0);
  Vector3d v1,v2;
  v1 = q1.tail(3);
  v2 = q2.tail(3);
  r << w1*w2-v1.dot(v2), v1.cross(v2)+w1*v2+w2*v1;
  dr.row(0) << w2,-v2.transpose(),w1,-v1.transpose(); 
  dr.row(1) << q2(1), q2(0), q2(3),-q2(2), q1(1), q1(0),-q1(3), q1(2);
  dr.row(2) << q2(2),-q2(3), q2(0), q2(1), q1(2), q1(3), q1(0),-q1(1);
  dr.row(3) << q2(3), q2(2),-q2(1), q2(0), q1(3),-q1(2), q1(1), q1(0);
}

void quatRotateVec(const Vector4d &q, const Vector3d &v, Vector3d &r)
{
  Vector4d q_times_v;
  Vector4d v_quat;
  Vector4d q_conj;
  Vector4d v_rot;

  v_quat << 0, v;
  quatProduct(q,v_quat,q_times_v);
  quatConjugate(q,q_conj);
  quatProduct(q_times_v,q_conj,v_rot);
  r << v_rot.bottomRows(3);
}
void quatRotateVec(const Vector4d &q, const Vector3d &v, Vector3d &r, Matrix<double,3,7> &dr)
{
  Vector4d q_times_v;
  Vector4d v_quat;
  Vector4d q_conj;
  Vector4d v_rot;
  Matrix<double,4,7> dq;
  Matrix<double,4,7> dv;
  Matrix<double,8,7> dqdv;
  dq << Matrix4d::Identity(), MatrixXd::Zero(4,3);
  dv << Matrix<double,4,7>::Zero();
  dv.bottomRightCorner(3,3) = Matrix3d::Identity();
  dqdv << dq, dv;
  Matrix<double,4,8> dq_times_v_tmp;
  Matrix4d dq_conj_tmp;
  Matrix<double,4,8> dv_rot_tmp;
  Matrix<double,4,7> dq_times_v;
  Matrix<double,4,7> dq_conj;
  Matrix<double,4,7> dv_rot;
  Matrix<double,8,7> dq_times_v_dq_conj;

  v_quat << 0, v;
  quatProduct(q,v_quat,q_times_v,dq_times_v_tmp);
  dq_times_v = dq_times_v_tmp*dqdv;
  quatConjugate(q,q_conj,dq_conj_tmp);
  dq_conj = dq_conj_tmp*dq;
  dq_times_v_dq_conj << dq_times_v, dq_conj;
  quatProduct(q_times_v,q_conj,v_rot,dv_rot_tmp);
  dv_rot = dv_rot_tmp*dq_times_v_dq_conj;
  r << v_rot.bottomRows(3);
  dr << dv_rot.bottomRows(3);
}

void quatDiff(const Vector4d &q1, const Vector4d &q2, Vector4d &r, Matrix<double,4,8> &dr)
{
  Vector4d q1_conj;
  q1_conj<<q1(0),-q1(1),-q1(2),-q1(3);
  quatProduct(q1_conj,q2,r,dr);
  dr.block(0,1,4,3) = -dr.block(0,1,4,3);
}

void quatDiffAxisInvar(const Vector4d &q1, const Vector4d &q2, const Vector3d &u, double &e, Matrix<double,1,11> &de)
{
  Vector4d r;
  Matrix<double,4,8> dr;
  quatDiff(q1,q2,r,dr);
  e = -2.0+2*r(0)*r(0)+2*pow(u(0)*r(1)+u(1)*r(2)+u(2)*r(3),2);
  de << 4*r(0)*dr.row(0)+4*u.transpose()*r.tail(3)*u.transpose()*dr.block(1,0,3,8),4*u.transpose()*r.tail(3)*r.tail(3).transpose();
}


