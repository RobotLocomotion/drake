#include "drakeQuatUtil.h"

using namespace Eigen;
void quatConjugate(Vector4d q, Vector4d &q_conj)
{
  q_conj<<q(0),-q(1),-q(2),-q(3);
}

void quatConjugate(Vector4d q, Vector4d &q_conj, Matrix4d &dq_conj)
{
  quatConjugate(q,q_conj);
  dq_conj = Matrix4d::Zero();
  dq_conj(1,1) = -1.0;
  dq_conj(2,2) = -1.0;
  dq_conj(3,3) = -1.0;
}
void quatProduct(Vector4d q1,Vector4d q2, Vector4d &r)
{
  double w1 = q1(0);
  double w2 = q2(0);
  Vector3d v1,v2;
  v1 = q1.tail(3);
  v2 = q2.tail(3);
  r << w1*w2-v1.dot(v2), v1.cross(v2)+w1*v2+w2*v1;
}

void quatProduct(Vector4d q1, Vector4d q2, Vector4d &r, Matrix<double,4,8> &dr)
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

void quatDiff(Vector4d q1, Vector4d q2, Vector4d &r, Matrix<double,4,8> &dr)
{
  Vector4d q1_conj;
  q1_conj<<q1(0),-q1(1),-q1(2),-q1(3);
  quatProduct(q1_conj,q2,r,dr);
  dr.block(0,1,4,3) = -dr.block(0,1,4,3);
}

void quatDiffAxisInvar(Vector4d q1, Vector4d q2, Vector3d u, double &e, Matrix<double,1,11> &de)
{
  Vector4d r;
  Matrix<double,4,8> dr;
  quatDiff(q1,q2,r,dr);
  e = -2.0+2*r(0)*r(0)+2*pow(u(0)*r(1)+u(1)*r(2)+u(2)*r(3),2);
  de << 4*r(0)*dr.row(0)+4*u.transpose()*r.tail(3)*u.transpose()*dr.block(1,0,3,8),4*u.transpose()*r.tail(3)*r.tail(3).transpose();
}

void quatTransform(Vector3d u, Vector3d v, Vector4d &quat)
{
  Vector3d quat_axis = u+v;
  double len_quat_axis = quat_axis.norm();
  quat << 0,quat_axis/len_quat_axis;
}

void quatTransform(Vector3d u, Vector3d v, Vector4d &quat, Matrix<double,4,6> &dquat)
{
  Vector3d quat_axis = u+v;
  double len_quat_axis = quat_axis.norm();
  quat << 0,quat_axis/len_quat_axis;
  Matrix<double,3,6> dquat_axis;
  dquat_axis<< Matrix3d::Identity(), Matrix3d::Identity();
  Matrix<double,1,6> dlen_quat_axis;
  dlen_quat_axis = quat_axis.transpose()*dquat_axis/len_quat_axis;
  dquat.row(0) = Matrix<double,1,6>::Zero();
  dquat.block(1,0,3,6) = (dquat_axis*len_quat_axis-quat_axis*dlen_quat_axis)/(len_quat_axis*len_quat_axis);
}
