#ifndef __DRAKE_GEOMETRY_UTIL_H__
#define __DRAKE_GEOMETRY_UTIL_H__

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <cstring>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <random>
#include "drakeGradientUtil.h"
#include "GradientVar.h"

const int TWIST_SIZE = 6;
const int QUAT_SIZE = 4;
const int EXPMAP_SIZE = 3;
const int HOMOGENEOUS_TRANSFORM_SIZE = 16;
const int AXIS_ANGLE_SIZE = 4;
const int SPACE_DIMENSION = 3;
const int RotmatSize = SPACE_DIMENSION * SPACE_DIMENSION;
const int RPY_SIZE = 3;

inline double angleDiff(double phi1, double phi2) {
  double d = phi2-phi1;
  if(d>0.0)
  {
    d = fmod(d+M_PI,2*M_PI)-M_PI;
  }
  else
  {
    d = fmod(d-M_PI,2*M_PI)+M_PI;
  }
  return d;
}

/*
 * quaternion methods
 */
inline Eigen::Vector4d quatConjugate(const Eigen::Vector4d &q) {
  Eigen::Vector4d q_conj;
  q_conj << q(0), -q(1), -q(2), -q(3);
  return q_conj;
}

inline Eigen::Matrix4d dquatConjugate() {
  Eigen::Matrix4d dq_conj = Eigen::Matrix4d::Identity();
  dq_conj(1, 1) = -1.0;
  dq_conj(2, 2) = -1.0;
  dq_conj(3, 3) = -1.0;
  return dq_conj;
}

inline Eigen::Vector4d quatProduct(const Eigen::Vector4d &q1, const Eigen::Vector4d &q2) {
  double w1 = q1(0);
  double w2 = q2(0);
  const auto& v1 = q1.tail<3>();
  const auto& v2 = q2.tail<3>();
  Eigen::Vector4d r;
  r << w1 * w2 - v1.dot(v2), v1.cross(v2) + w1 * v2 + w2 * v1;
  return r;
}

inline Eigen::Matrix<double, 4, 8> dquatProduct(const Eigen::Vector4d &q1,const Eigen::Vector4d &q2) {
  double w1 = q1(0);
  double w2 = q2(0);
  const auto& v1 = q1.tail<3>();
  const auto& v2 = q2.tail<3>();

  Eigen::Matrix<double, 4, 8> dr;
  dr.row(0) << w2, -v2.transpose(), w1, -v1.transpose();
  dr.row(1) << q2(1), q2(0), q2(3), -q2(2), q1(1), q1(0), -q1(3), q1(2);
  dr.row(2) << q2(2), -q2(3), q2(0), q2(1), q1(2), q1(3), q1(0), -q1(1);
  dr.row(3) << q2(3), q2(2), -q2(1), q2(0), q1(3), -q1(2), q1(1), q1(0);
  return dr;
}

inline Eigen::Vector3d quatRotateVec(const Eigen::Vector4d &q, const Eigen::Vector3d &v) {
  using namespace Eigen;
  Vector4d v_quat;
  v_quat << 0, v;
  Vector4d q_times_v = quatProduct(q, v_quat);
  Vector4d q_conj = quatConjugate(q);
  Vector4d v_rot = quatProduct(q_times_v, q_conj);
  Vector3d r = v_rot.bottomRows<3>();
  return r;
}

inline Eigen::Matrix<double, 3, 7> dquatRotateVec(const Eigen::Vector4d &q, const Eigen::Vector3d &v) {
  using namespace Eigen;
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

  Matrix<double, 4, 7> dq_conj = dquatConjugate() * dq;
  Matrix<double, 8, 7> dq_times_v_dq_conj;
  dq_times_v_dq_conj << dq_times_v, dq_conj;
  Matrix<double, 4, 8> dv_rot_tmp = dquatProduct(q_times_v, quatConjugate(q));
  Matrix<double, 4, 7> dv_rot = dv_rot_tmp * dq_times_v_dq_conj;
  Matrix<double, 3, 7> dr = dv_rot.bottomRows(3);
  return dr;
}

inline Eigen::Vector4d quatDiff(const Eigen::Vector4d &q1, const Eigen::Vector4d &q2) {
  return quatProduct(quatConjugate(q1), q2);
}

inline Eigen::Matrix<double, 4, 8> dquatDiff(const Eigen::Vector4d &q1, const Eigen::Vector4d &q2) {
  auto dr = dquatProduct(quatConjugate(q1), q2);
  dr.block<4, 3>(0, 1) = -dr.block<4, 3>(0, 1);
  return dr;
}

inline double quatDiffAxisInvar(const Eigen::Vector4d &q1, const Eigen::Vector4d &q2, const Eigen::Vector3d &u) {
  Eigen::Vector4d r = quatDiff(q1, q2);
  double e = -2.0 + 2 * r(0) * r(0) + 2 * pow(u(0) * r(1) + u(1) * r(2) + u(2) * r(3), 2);
  return e;
}

inline Eigen::Matrix<double, 1, 11> dquatDiffAxisInvar(const Eigen::Vector4d &q1, const Eigen::Vector4d &q2, const Eigen::Vector3d &u)
{
  using namespace Eigen;
  Vector4d r = quatDiff(q1, q2);
  Matrix<double, 4, 8> dr = dquatDiff(q1, q2);
  Matrix<double, 1, 11> de;
  const auto& rvec = r.tail<3>();
  de << 4.0 * r(0) * dr.row(0) + 4.0 * u.transpose() * rvec *u.transpose() * dr.block<3, 8>(1, 0), 4.0 * u.transpose() * rvec * rvec.transpose();
  return de;
}

inline double quatNorm(const Eigen::Vector4d& q) {
  return std::acos(q(0));
}

inline Eigen::Vector4d slerp(const Eigen::Vector4d& q1, const Eigen::Vector4d& q2, double interpolation_parameter) {
  /*
   * Q = slerp(q1, q2, f) Spherical linear interpolation between two quaternions
   *   This function uses the implementation given in Algorithm 8 of [1].
   *
   * @param q1   Initial quaternion (w, x, y, z)
   * @param q2   Final quaternion (w, x, y, z)
   * @param f    Interpolation parameter between 0 and 1 (inclusive)
   * @retval Q   Interpolated quaternion(s). 4-by-1 vector.
   *
   * [1] Kuffner, J.J., "Effective sampling and distance metrics for 3D rigid
   * body path planning," Robotics and Automation, 2004. Proceedings. ICRA '04.
   * 2004 IEEE International Conference on , vol.4, no., pp.3993,3998 Vol.4,
   * April 26-May 1, 2004
   * doi: 10.1109/ROBOT.2004.1308895
   */

  // Compute the quaternion inner product
  double lambda = (q1.transpose() * q2).value();
  int q2_sign;
  if (lambda < 0.0) {
    // The quaternions are pointing in opposite directions, so use the equivalent alternative representation for q2
    lambda = -lambda;
    q2_sign = -1;
  }
  else {
    q2_sign = 1;
  }

  // Calculate interpolation factors
  // TODO: do we really want an epsilon so small?
  double r, s;
  if (std::abs(1.0 - lambda) < std::numeric_limits<double>::epsilon()) {
    // The quaternions are nearly parallel, so use linear interpolation
    r = 1.0 - interpolation_parameter;
    s = interpolation_parameter;
  }
  else {
    double alpha = std::acos(lambda);
    double gamma = 1.0 / std::sin(alpha);
    r = std::sin((1.0 - interpolation_parameter) * alpha) * gamma;
    s = std::sin(interpolation_parameter * alpha) * gamma;
  }

  Eigen::Vector4d ret = q1 * r;
  ret += q2_sign * q2 * s;
  return ret;
}


/*
 * quat2x
 */
template <typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 4, 1> quat2axis(const Eigen::MatrixBase<Derived>& q) {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 4);
  auto q_normalized = q.normalized();
  auto s = sqrt(1.0 - q_normalized(0) * q_normalized(0)) + std::numeric_limits<typename Derived::Scalar>::epsilon();
  Eigen::Matrix<typename Derived::Scalar, 4, 1> a;

  a << q_normalized.template tail<3>() / s, 2.0 * std::acos(q_normalized(0));
  return a;
}

template <typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3> quat2rotmat(const Eigen::MatrixBase<Derived>& q) {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 4);
  auto q_normalized = q.normalized();
  auto w = q_normalized(0);
  auto x = q_normalized(1);
  auto y = q_normalized(2);
  auto z = q_normalized(3);

  Eigen::Matrix<typename Derived::Scalar, 3, 3> M;
  M.row(0) << w * w + x * x - y * y - z * z, 2.0 * x * y - 2.0 * w * z, 2.0 * x * z + 2.0 * w * y;
  M.row(1) << 2.0 * x * y + 2.0 * w * z, w * w + y * y - x * x - z * z, 2.0 * y * z - 2.0 * w * x;
  M.row(2) << 2.0 * x * z - 2.0 * w * y, 2.0 * y * z + 2.0 * w * x, w * w + z * z - x * x - y * y;

  return M;
}

template <typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 1> quat2rpy(const Eigen::MatrixBase<Derived>& q) {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 4);
  auto q_normalized = q.normalized();
  auto w = q_normalized(0);
  auto x = q_normalized(1);
  auto y = q_normalized(2);
  auto z = q_normalized(3);

  Eigen::Matrix<typename Derived::Scalar, 3, 1> ret;
  ret << std::atan2(2.0*(w*x + y*z), w*w + z*z -(x*x +y*y)),
          std::asin(2.0*(w*y - z*x)),
          std::atan2(2.0*(w*z + x*y), w*w + x*x-(y*y+z*z));
  return ret;
}

template <typename Derived>
inline Eigen::Quaternion<typename Derived::Scalar> quat2eigenQuaternion(const Eigen::MatrixBase<Derived> &q)
{
  // The Eigen Quaterniond constructor when used with 4 arguments, uses the (w, x, y, z) ordering, just as we do.
  // HOWEVER: when the constructor is called on a 4-element Vector, the elements must be in (x, y, z, w) order.
  // So, the following two calls will give you the SAME quaternion:
  // Quaternion<double>(q(0), q(1), q(2), q(3));
  // Quaternion<double>(Vector4d(q(3), q(0), q(1), q(2)))
  // which is gross and will cause you much pain.
  // see: http://eigen.tuxfamily.org/dox/classEigen_1_1Quaternion.html#a91b6ea2cac13ab2d33b6e74818ee1490
  //
  // This method takes a nice, normal (w, x, y, z) order vector and gives you the Quaternion you expect.
  return Eigen::Quaternion<typename Derived::Scalar>(q(0), q(1), q(2), q(3));
}

/*
 * axis2x
 */
template <typename Derived>
inline Eigen::Vector4d axis2quat(const Eigen::MatrixBase<Derived>& a) {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 4);
  auto axis = a.template head<3>();
  auto angle = a(3);
  auto arg = 0.5 * angle;
  auto c = std::cos(arg);
  auto s = std::sin(arg);
  Eigen::Vector4d ret;
  ret << c, s * axis;
  return ret;
}

template <typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3> axis2rotmat(const Eigen::MatrixBase<Derived>& a) {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 4);
  const auto& axis = (a.template head<3>())/(a.template head<3>()).norm();
  const auto& theta = a(3);
  auto x = axis(0);
  auto y = axis(1);
  auto z = axis(2);
  auto ctheta = std::cos(theta);
  auto stheta = std::sin(theta);
  auto c = 1 - ctheta;
  Eigen::Matrix<typename Derived::Scalar, 3, 3> R;
  R <<
  ctheta + x * x * c , x * y * c - z * stheta, x * z * c + y * stheta,
          y * x * c + z * stheta, ctheta + y * y * c, y * z * c - x * stheta,
          z * x * c - y * stheta, z * y * c + x * stheta, ctheta + z * z * c;

  return R;
}

template <typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 1> axis2rpy(const Eigen::MatrixBase<Derived>& a) {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 4);
  return quat2rpy(axis2quat(a));
};

/*
 * expmap2x
 */
inline Eigen::Vector4d expmap2quatNonDegenerate(const Eigen::Ref<const Eigen::Vector3d>& v, double theta)
{
  double t2;
  double t3;
  double t4;
  Eigen::Vector4d q;

  t2 = theta*(1.0/2.0);
  t3 = 1.0/theta;
  t4 = sin(t2);
  q(0,0) = cos(t2);
  q(1,0) = t3*t4*v(0);
  q(2,0) = t3*t4*v(1);
  q(3,0) = t3*t4*v(2);

  return q;
}

inline Eigen::Vector4d expmap2quatDegenerate(const Eigen::Ref<const Eigen::Vector3d>& v, double theta)
{
  double t2;
  double t3;
  double t4;
  Eigen::Vector4d q;

  t2 = theta*theta;
  t3 = t2*8.0E1;
  t4 = t3-1.92E3;
  q(0,0) = t2*(-1.0/8.0)+1.0;
  q(1,0) = t4*v(0)*(-2.604166666666667E-4);
  q(2,0) = t4*v(1)*(-2.604166666666667E-4);
  q(3,0) = t4*v(2)*(-2.604166666666667E-4);

  return q;
}

inline Eigen::Matrix<double, 4, 3> dexpmap2quatNonDegenerate(const Eigen::Ref<const Eigen::Vector3d>& v, double theta)
{
  double t10;
  double t11;
  double t12;
  double t13;
  double t14;
  double t2;
  double t3;
  double t4;
  double t5;
  double t6;
  double t7;
  double t8;
  double t9;
  Eigen::Matrix<double, 4, 3> q;

  t2 = 1.0/theta;
  t3 = theta*(1.0/2.0);
  t4 = sin(t3);
  t5 = v(0)*v(0);
  t6 = 1.0/(theta*theta*theta);
  t7 = cos(t3);
  t8 = t4*2.0;
  t10 = t7*theta;
  t9 = t8-t10;
  t11 = theta*theta;
  t12 = t4*t11*2.0;
  t13 = v(1)*v(1);
  t14 = v(2)*v(2);
  q(0,0) = t2*t4*v(0)*(-1.0/2.0);
  q(0,1) = t2*t4*v(1)*(-1.0/2.0);
  q(0,2) = t2*t4*v(2)*(-1.0/2.0);
  q(1,0) = t6*(t12-t4*t5*2.0+t5*t7*theta)*(1.0/2.0);
  q(1,1) = t6*t9*v(0)*v(1)*(-1.0/2.0);
  q(1,2) = t6*t9*v(0)*v(2)*(-1.0/2.0);
  q(2,0) = t6*t9*v(0)*v(1)*(-1.0/2.0);
  q(2,1) = t6*(t12-t4*t13*2.0+t7*t13*theta)*(1.0/2.0);
  q(2,2) = t6*t9*v(1)*v(2)*(-1.0/2.0);
  q(3,0) = t6*t9*v(0)*v(2)*(-1.0/2.0);
  q(3,1) = t6*t9*v(1)*v(2)*(-1.0/2.0);
  q(3,2) = t6*(t12-t4*t14*2.0+t7*t14*theta)*(1.0/2.0);

  return q;
}

inline Eigen::Matrix<double, 4, 3> dexpmap2quatDegenerate(const Eigen::Ref<const Eigen::Vector3d>& v, double theta)
{
  double t10;
  double t11;
  double t2;
  double t3;
  double t4;
  double t5;
  double t6;
  double t7;
  double t8;
  double t9;
  Eigen::Matrix<double, 4, 3> q;

  t2 = theta*theta;
  t3 = t2*8.0E1;
  t4 = t3-1.92E3;
  t5 = v(0)*v(0);
  t6 = t2-4.0E1;
  t7 = t6*v(0)*v(1)*(1.0/9.6E2);
  t8 = v(1)*v(1);
  t9 = t6*v(0)*v(2)*(1.0/9.6E2);
  t10 = t6*v(1)*v(2)*(1.0/9.6E2);
  t11 = v(2)*v(2);
  q(0,0) = t4*v(0)*1.302083333333333E-4;
  q(0,1) = t4*v(1)*1.302083333333333E-4;
  q(0,2) = t4*v(2)*1.302083333333333E-4;
  q(1,0) = t2*(-1.0/4.8E1)-t5*(1.0/2.4E1)+t2*t5*(1.0/9.6E2)+1.0/2.0;
  q(1,1) = t7;
  q(1,2) = t9;
  q(2,0) = t7;
  q(2,1) = t2*(-1.0/4.8E1)-t8*(1.0/2.4E1)+t2*t8*(1.0/9.6E2)+1.0/2.0;
  q(2,2) = t10;
  q(3,0) = t9;
  q(3,1) = t10;
  q(3,2) = t2*(-1.0/4.8E1)-t11*(1.0/2.4E1)+t2*t11*(1.0/9.6E2)+1.0/2.0;

  return q;
}

inline Eigen::Matrix<double, 12, 3> ddexpmap2quatNonDegenerate(const Eigen::Ref<const Eigen::Vector3d>& v, double theta) {
  double t10;
  double t11;
  double t12;
  double t13;
  double t14;
  double t15;
  double t16;
  double t17;
  double t18;
  double t19;
  double t2;
  double t20;
  double t21;
  double t22;
  double t23;
  double t24;
  double t25;
  double t26;
  double t27;
  double t28;
  double t29;
  double t3;
  double t30;
  double t31;
  double t32;
  double t33;
  double t34;
  double t35;
  double t36;
  double t37;
  double t38;
  double t39;
  double t4;
  double t40;
  double t41;
  double t42;
  double t43;
  double t44;
  double t45;
  double t46;
  double t47;
  double t48;
  double t49;
  double t5;
  double t50;
  double t51;
  double t52;
  double t53;
  double t54;
  double t55;
  double t6;
  double t7;
  double t8;
  double t9;
  Eigen::Matrix<double, 12, 3> q;

  t2 = theta*(1.0/2.0);
  t3 = sin(t2);
  t4 = v(0)*v(0);
  t5 = 1.0/(theta*theta*theta);
  t6 = cos(t2);
  t7 = t3*2.0;
  t15 = t6*theta;
  t8 = t7-t15;
  t9 = theta*theta;
  t10 = 1.0/(theta*theta*theta*theta*theta);
  t11 = t3*t4*t9;
  t12 = t4*t6*theta*6.0;
  t13 = t3*t9*4.0;
  t35 = t6*t9*theta*2.0;
  t14 = t11+t12+t13-t35-t3*t4*1.2E1;
  t16 = 1.0/theta;
  t17 = 1.0/(theta*theta*theta*theta);
  t18 = t8*t17*v(0)*v(1)*(3.0/2.0);
  t19 = 1.0/(theta*theta);
  t29 = t3*t19*v(0)*v(1)*(1.0/4.0);
  t20 = t18-t29;
  t21 = t6*theta*6.0;
  t22 = t3*t9;
  t34 = t3*1.2E1;
  t23 = t21+t22-t34;
  t24 = t3*t19*v(0)*v(2)*(1.0/4.0);
  t46 = t8*t17*v(0)*v(2)*(3.0/2.0);
  t25 = t24-t46;
  t26 = t5*t8*v(0)*v(1)*(1.0/4.0);
  t27 = t3*t9*2.0;
  t28 = v(1)*v(1);
  t30 = t16*t20*v(0);
  t42 = t5*t8*v(1)*(1.0/2.0);
  t31 = t30-t42;
  t32 = t16*t20*v(1);
  t48 = t5*t8*v(0)*(1.0/2.0);
  t33 = t32-t48;
  t36 = t3*t9*1.2E1;
  t37 = t3*t9*t28;
  t38 = t6*t28*theta*6.0;
  t39 = t13-t35+t37+t38-t3*t28*1.2E1;
  t40 = t3*t19*v(1)*v(2)*(1.0/4.0);
  t49 = t8*t17*v(1)*v(2)*(3.0/2.0);
  t41 = t40-t49;
  t43 = t5*t8*v(0)*v(2)*(1.0/4.0);
  t44 = t5*t8*v(1)*v(2)*(1.0/4.0);
  t45 = v(2)*v(2);
  t50 = t5*t8*v(2)*(1.0/2.0);
  t47 = -t50-t16*t25*v(0);
  t51 = -t42-t16*t41*v(2);
  t52 = t3*t9*t45;
  t53 = t6*t45*theta*6.0;
  t55 = t3*t45*1.2E1;
  t54 = t13-t35+t52+t53-t55;
  q(0,0) = t5*(t27-t3*t4*2.0+t4*t6*theta)*(-1.0/4.0);
  q(0,1) = t26;
  q(0,2) = t43;
  q(1,0) = t10*v(0)*(t11+t12+t36-t3*t4*1.2E1-t6*t9*theta*6.0)*(-1.0/4.0);
  q(1,1) = t10*t14*v(1)*(-1.0/4.0);
  q(1,2) = t10*t14*v(2)*(-1.0/4.0);
  q(2,0) = t31;
  q(2,1) = t33;
  q(2,2) = t10*t23*v(0)*v(1)*v(2)*(-1.0/4.0);
  q(3,0) = t47;
  q(3,1) = t10*t23*v(0)*v(1)*v(2)*(-1.0/4.0);
  q(3,2) = t5*t8*v(0)*(-1.0/2.0)-t16*t25*v(2);
  q(4,0) = t26;
  q(4,1) = t5*(t27-t3*t28*2.0+t6*t28*theta)*(-1.0/4.0);
  q(4,2) = t44;
  q(5,0) = t31;
  q(5,1) = t33;
  q(5,2) = t10*t23*v(0)*v(1)*v(2)*(-1.0/4.0);
  q(6,0) = t10*t39*v(0)*(-1.0/4.0);
  q(6,1) = t10*v(1)*(t36+t37+t38-t3*t28*1.2E1-t6*t9*theta*6.0)*(-1.0/4.0);
  q(6,2) = t10*t39*v(2)*(-1.0/4.0);
  q(7,0) = t10*t23*v(0)*v(1)*v(2)*(-1.0/4.0);
  q(7,1) = t5*t8*v(2)*(-1.0/2.0)-t16*t41*v(1);
  q(7,2) = t51;
  q(8,0) = t43;
  q(8,1) = t44;
  q(8,2) = t5*(t27-t3*t45*2.0+t6*t45*theta)*(-1.0/4.0);
  q(9,0) = t47;
  q(9,1) = t10*t23*v(0)*v(1)*v(2)*(-1.0/4.0);
  q(9,2) = -t48-t16*t25*v(2);
  q(10,0) = t10*t23*v(0)*v(1)*v(2)*(-1.0/4.0);
  q(10,1) = -t50-t16*t41*v(1);
  q(10,2) = t51;
  q(11,0) = t10*t54*v(0)*(-1.0/4.0);
  q(11,1) = t10*t54*v(1)*(-1.0/4.0);
  q(11,2) = t10*v(2)*(t36+t52+t53-t55-t6*t9*theta*6.0)*(-1.0/4.0);

  return q;
}

inline Eigen::Matrix<double, 12, 3> ddexpmap2quatDegenerate(const Eigen::Ref<const Eigen::Vector3d>& v, double theta) {
  double t10;
  double t11;
  double t12;
  double t13;
  double t14;
  double t15;
  double t16;
  double t17;
  double t18;
  double t19;
  double t2;
  double t20;
  double t21;
  double t22;
  double t3;
  double t4;
  double t5;
  double t6;
  double t7;
  double t8;
  double t9;
  Eigen::Matrix<double, 12, 3> q;

  t2 = theta*theta;
  t3 = v(0)*v(0);
  t4 = t2-4.0E1;
  t5 = t3*3.2E1;
  t6 = t2*1.6E1;
  t9 = t2*t3;
  t7 = v(1)*(t5+t6-t9-6.4E2)*6.510416666666667E-5;
  t8 = v(1)*v(1);
  t10 = t2-3.2E1;
  t11 = v(2)*v(2);
  t12 = t2*(1.0/9.6E1);
  t13 = t8*3.2E1;
  t15 = t2*t8;
  t14 = v(0)*(t6+t13-t15-6.4E2)*6.510416666666667E-5;
  t16 = t2*4.8E1;
  t17 = v(2)*(t6+t13-t15-6.4E2)*6.510416666666667E-5;
  t18 = t11*3.2E1;
  t19 = v(2)*(t5+t6-t9-6.4E2)*6.510416666666667E-5;
  t21 = t2*t11;
  t20 = v(0)*(t6+t18-t21-6.4E2)*6.510416666666667E-5;
  t22 = v(1)*(t6+t18-t21-6.4E2)*6.510416666666667E-5;
  q(0,0) = t3*(1.0/4.8E1)+t12-t2*t3*5.208333333333333E-4-1.0/4.0;
  q(0,1) = t4*v(0)*v(1)*(-5.208333333333333E-4);
  q(0,2) = t4*v(0)*v(2)*(-5.208333333333333E-4);
  q(1,0) = v(0)*(t5+t16-t2*t3-1.92E3)*6.510416666666667E-5;
  q(1,1) = t7;
  q(1,2) = v(2)*(t5+t6-t2*t3-6.4E2)*6.510416666666667E-5;
  q(2,0) = t7;
  q(2,1) = t14;
  q(2,2) = t10*v(0)*v(1)*v(2)*(-6.510416666666667E-5);
  q(3,0) = t19;
  q(3,1) = t10*v(0)*v(1)*v(2)*(-6.510416666666667E-5);
  q(3,2) = t20;
  q(4,0) = t4*v(0)*v(1)*(-5.208333333333333E-4);
  q(4,1) = t8*(1.0/4.8E1)+t12-t2*t8*5.208333333333333E-4-1.0/4.0;
  q(4,2) = t4*v(1)*v(2)*(-5.208333333333333E-4);
  q(5,0) = t7;
  q(5,1) = t14;
  q(5,2) = t10*v(0)*v(1)*v(2)*(-6.510416666666667E-5);
  q(6,0) = t14;
  q(6,1) = v(1)*(t13-t15+t16-1.92E3)*6.510416666666667E-5;
  q(6,2) = t17;
  q(7,0) = t10*v(0)*v(1)*v(2)*(-6.510416666666667E-5);
  q(7,1) = t17;
  q(7,2) = v(1)*(t6+t18-t2*t11-6.4E2)*6.510416666666667E-5;
  q(8,0) = t4*v(0)*v(2)*(-5.208333333333333E-4);
  q(8,1) = t4*v(1)*v(2)*(-5.208333333333333E-4);
  q(8,2) = t11*(1.0/4.8E1)+t12-t2*t11*5.208333333333333E-4-1.0/4.0;
  q(9,0) = t19;
  q(9,1) = t10*v(0)*v(1)*v(2)*(-6.510416666666667E-5);
  q(9,2) = t20;
  q(10,0) = t10*v(0)*v(1)*v(2)*(-6.510416666666667E-5);
  q(10,1) = t17;
  q(10,2) = t22;
  q(11,0) = t20;
  q(11,1) = t22;
  q(11,2) = v(2)*(t16+t18-t21-1.92E3)*6.510416666666667E-5;

  return q;
}


template <typename Derived>
inline GradientVar<typename Derived::Scalar, QUAT_SIZE, 1> expmap2quat(const Eigen::MatrixBase<Derived>& v, const int gradient_order) {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 3);
  GradientVar<typename Derived::Scalar, QUAT_SIZE, 1> ret(QUAT_SIZE, 1, EXPMAP_SIZE,gradient_order);
  auto theta = v.norm();
  if (theta < pow(std::numeric_limits<typename Derived::Scalar>::epsilon(),0.25)) {
    ret.value() = expmap2quatDegenerate(v, theta);
    if(gradient_order>0)
    {
      ret.gradient().value() = dexpmap2quatDegenerate(v, theta);
      if(gradient_order>1)
      {
        ret.gradient().gradient().value() = ddexpmap2quatDegenerate(v, theta);
        if(gradient_order>2)
        {
          throw std::runtime_error("expmap2quat does not support gradient order larger than 2");
        }
      }
    }
  } else {
    ret.value() = expmap2quatNonDegenerate(v, theta);
    if(gradient_order>0)
    {
      ret.gradient().value() = dexpmap2quatNonDegenerate(v, theta);
      if(gradient_order>1)
      {
        ret.gradient().gradient().value() = ddexpmap2quatNonDegenerate(v, theta);
        if(gradient_order>2)
        {
          throw std::runtime_error("expmap2quat does not support gradient order larger than 2");
        }
      }
    }
  }
  return ret;
}

/*
 * rotmat2x
 */
template <typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 4, 1> rotmat2axis(const Eigen::MatrixBase<Derived>& R) {
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 3, 3);

  typename Derived::Scalar theta = std::acos((R.trace() - 1.0) / 2.0);
  Eigen::Vector4d a;
  if (theta > std::numeric_limits<typename Derived::Scalar>::epsilon()) {
    a << R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1), theta;
    a.head<3>() *= 1.0 / (2.0 * std::sin(theta));
  }
  else {
    a << 1.0, 0.0, 0.0, 0.0;
  }
  return a;
};

template <typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 4, 1> rotmat2quat(const Eigen::MatrixBase<Derived>& M) {
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 3, 3);
  using namespace std;
  using namespace Eigen;

  Matrix<typename Derived::Scalar, 4, 3> A;
  A.row(0) << 1.0, 1.0, 1.0;
  A.row(1) << 1.0, -1.0, -1.0;
  A.row(2) << -1.0, 1.0, -1.0;
  A.row(3) << -1.0, -1.0, 1.0;
  Matrix<typename Derived::Scalar, 4, 1> B = A * M.diagonal();
  typename Matrix<typename Derived::Scalar, 4, 1>::Index ind, max_col;
  typename Derived::Scalar val = B.maxCoeff(&ind, &max_col);

  typename Derived::Scalar w, x, y, z;
  switch (ind) {
    case 0: {
      // val = trace(M)
      w = sqrt(1.0 + val) / 2.0;
      typename Derived::Scalar w4 = w * 4.0;
      x = (M(2, 1) - M(1, 2)) / w4;
      y = (M(0, 2) - M(2, 0)) / w4;
      z = (M(1, 0) - M(0, 1)) / w4;
      break;
    }
    case 1: {
      // val = M(1,1) - M(2,2) - M(3,3)
      double s = 2.0 * sqrt(1.0 + val);
      w = (M(2, 1) - M(1, 2)) / s;
      x = 0.25 * s;
      y = (M(0, 1) + M(1, 0)) / s;
      z = (M(0, 2) + M(2, 0)) / s;
      break;
    }
    case 2: {
      //  % val = M(2,2) - M(1,1) - M(3,3)
      double s = 2.0 * (sqrt(1.0 + val));
      w = (M(0, 2) - M(2, 0)) / s;
      x = (M(0, 1) + M(1, 0)) / s;
      y = 0.25 * s;
      z = (M(1, 2) + M(2, 1)) / s;
      break;
    }
    default: {
      // val = M(3,3) - M(2,2) - M(1,1)
      double s = 2.0 * (sqrt(1.0 + val));
      w = (M(1, 0) - M(0, 1)) / s;
      x = (M(0, 2) + M(2, 0)) / s;
      y = (M(1, 2) + M(2, 1)) / s;
      z = 0.25 * s;
      break;
    }
  }

  Eigen::Matrix<typename Derived::Scalar, 4, 1> q;
  q << w, x, y, z;
  return q;
}

template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 1> rotmat2rpy(const Eigen::MatrixBase<Derived>& R) {
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 3, 3);
  using namespace std;

  Eigen::Matrix<typename Derived::Scalar, 3, 1> rpy;
  rpy << atan2(R(2, 1), R(2, 2)), atan2(-R(2, 0), sqrt(pow(R(2, 1), 2.0) + pow(R(2, 2), 2.0))), atan2(R(1, 0), R(0, 0));
  return rpy;
}

template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, Eigen::Dynamic, 1> rotmat2Representation(const Eigen::MatrixBase<Derived>& R, int rotation_type) {
  typedef typename Derived::Scalar Scalar;
  Eigen::Matrix<Scalar, Eigen::Dynamic, 1> ret;
  switch (rotation_type) {
    case 0:
      return Eigen::Matrix<Scalar, Eigen::Dynamic, 1>(0, 1);
    case 1:
      return rotmat2rpy(R);
    case 2:
      return rotmat2quat(R);
    default:
      throw std::runtime_error("rotation representation type not recognized");
  }
}

inline int rotationRepresentationSize(int rotation_type) {
  switch (rotation_type) {
    case 0:
      return 0;
      break;
    case 1:
      return 3;
      break;
    case 2:
      return 4;
      break;
    default:
      throw std::runtime_error("rotation representation type not recognized");
  }
}

template<typename Scalar>
inline GradientVar<Scalar, Eigen::Dynamic, 1> rotmat2Representation(const GradientVar<Scalar, SPACE_DIMENSION, SPACE_DIMENSION>& R, int rotation_type) {
  GradientVar<Scalar, Eigen::Dynamic, 1> ret(rotationRepresentationSize(rotation_type), 1, R.getNumVariables(), R.maxOrder());
  switch (rotation_type) {
    case 0:
      // empty matrix, already done
      break;
    case 1:
      ret.value() = rotmat2rpy(R.value());
      if (R.hasGradient()) {
        ret.gradient().value() = drotmat2rpy(R.value(), R.gradient().value());
      }
      break;
    case 2:
      ret.value() = rotmat2quat(R.value());
      if (R.hasGradient()) {
        ret.gradient().value() = drotmat2quat(R.value(), R.gradient().value());
      }
      break;
    default:
      throw std::runtime_error("rotation representation type not recognized");
  }
  return ret;
}


/*
 * rpy2x
 */

template <typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 4, 1> rpy2quat(const Eigen::MatrixBase<Derived>& rpy) {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 3);
  auto rpy_2 = (rpy / 2.0).array();
  auto s = rpy_2.sin();
  auto c = rpy_2.cos();

  Eigen::Vector4d q;
  q << c(0)*c(1)*c(2) + s(0)*s(1)*s(2),
          s(0)*c(1)*c(2) - c(0)*s(1)*s(2),
          c(0)*s(1)*c(2) + s(0)*c(1)*s(2),
          c(0)*c(1)*s(2) - s(0)*s(1)*c(2);

  q /= q.norm() + std::numeric_limits<typename Derived::Scalar>::epsilon();
  return q;
}

template <typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 4, 1> rpy2axis(const Eigen::MatrixBase<Derived>& rpy) {
  return quat2axis(rpy2quat(rpy));
}

template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3> rpy2rotmat(const Eigen::MatrixBase<Derived>& rpy) {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 3);
  auto rpy_array = rpy.array();
  auto s = rpy_array.sin();
  auto c = rpy_array.cos();

  Eigen::Matrix<typename Derived::Scalar, 3, 3> R;
  R.row(0) << c(2) * c(1), c(2) * s(1) * s(0) - s(2) * c(0), c(2) * s(1) * c(0) + s(2) * s(0);
  R.row(1) << s(2) * c(1), s(2) * s(1) * s(0) + c(2) * c(0), s(2) * s(1) * c(0) - c(2) * s(0);
  R.row(2) << -s(1), c(1) * s(0), c(1) * c(0);

  return R;
}

template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 9, 3> drpy2rotmat(const Eigen::MatrixBase<Derived>& rpy) {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 3);
  auto rpy_array = rpy.array();
  auto s = rpy_array.sin();
  auto c = rpy_array.cos();

  Eigen::Matrix<typename Derived::Scalar, 9, 3> dR;
  dR.row(0) << 0, c(2)*-s(1), c(1)*-s(2);
  dR.row(1) << 0, -s(1)*s(2), c(2)*c(1);
  dR.row(2) << 0, -c(1), 0;
  dR.row(3) << c(2)*s(1)*c(0)-s(2)*-s(0), c(2)*c(1)*s(0), -s(2)*s(1)*s(0)-c(2)*c(0);
  dR.row(4) << s(2)*s(1)*c(0)+c(2)*-s(0), s(2)*c(1)*s(0), c(2)*s(1)*s(0)-s(2)*c(0);
  dR.row(5) << c(1)*c(0), -s(1)*s(0),0;
  dR.row(6) << c(2)*s(1)*-s(0)+s(2)*c(0), c(2)*c(1)*c(0), -s(2)*s(1)*c(0)+c(2)*s(0);
  dR.row(7) << s(2)*s(1)*-s(0)-c(2)*c(0), s(2)*c(1)*c(0), c(2)*s(1)*c(0)+s(2)*s(0);
  dR.row(8) << c(1)*-s(0), -s(1)*c(0), 0;

  return dR;
}

inline Eigen::Matrix3d rotz(double theta) {
  // returns 3D rotation matrix (about the z axis)
  Eigen::Matrix3d M;
  double c=cos(theta);
  double s=sin(theta);
  M << c,-s, 0,
          s, c, 0,
          0, 0, 1;
  return M;
}

inline void rotz(double theta, Eigen::Matrix3d &M, Eigen::Matrix3d &dM, Eigen::Matrix3d &ddM) {
  double c=cos(theta), s=sin(theta);
  M << c,-s,0, s,c,0, 0,0,1;
  dM << -s,-c,0, c,-s,0, 0,0,0;
  ddM << -c,s,0, -s,-c,0, 0,0,0;
}


/*
 * Random angles
 */

inline Eigen::Vector4d uniformlyRandomAxisAngle(std::default_random_engine& generator) {
  std::normal_distribution<double> normal;
  std::uniform_real_distribution<double> uniform(-M_PI, M_PI);
  double angle = uniform(generator);
  Eigen::Vector3d axis = Eigen::Vector3d(normal(generator), normal(generator), normal(generator));
  axis.normalize();
  Eigen::Vector4d a;
  a << axis, angle;
  return a;
}

inline Eigen::Vector4d uniformlyRandomQuat(std::default_random_engine& generator) {
  return axis2quat(uniformlyRandomAxisAngle(generator));
}

inline Eigen::Matrix3d uniformlyRandomRotmat(std::default_random_engine& generator) {
  return axis2rotmat(uniformlyRandomAxisAngle(generator));
}

inline Eigen::Vector3d uniformlyRandomRPY(std::default_random_engine& generator) {
  return axis2rpy(uniformlyRandomAxisAngle(generator));
}



/*
 * cross product related
 */
template <typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3> vectorToSkewSymmetric(const Eigen::MatrixBase<Derived>& p);

template <typename DerivedA, typename DerivedB>
inline Eigen::Matrix<typename DerivedA::Scalar, 3, Eigen::Dynamic> dcrossProduct(
    const Eigen::MatrixBase<DerivedA>& a,
    const Eigen::MatrixBase<DerivedB>& b,
    const typename Gradient<DerivedA, Eigen::Dynamic>::type& da,
    const typename Gradient<DerivedB, Eigen::Dynamic>::type& db) {
  Eigen::Matrix<typename DerivedA::Scalar, 3, Eigen::Dynamic> ret(3, da.cols());
  ret.noalias() = da.colwise().cross(b);
  ret.noalias() -= db.colwise().cross(a);
  return ret;
}


// NOTE: not reshaping second derivative to Matlab geval output format!
template <typename Derived>
inline void normalizeVec(
    const Eigen::MatrixBase<Derived>& x,
    typename Derived::PlainObject& x_norm,
    typename Gradient<Derived, Derived::RowsAtCompileTime, 1>::type* dx_norm = nullptr,
    typename Gradient<Derived, Derived::RowsAtCompileTime, 2>::type* ddx_norm = nullptr) {
  typename Derived::Scalar xdotx = x.squaredNorm();
  typename Derived::Scalar norm_x = sqrt(xdotx);
  x_norm = x / norm_x;

  if (dx_norm) {
    dx_norm->setIdentity(x.rows(), x.rows());
    (*dx_norm) -= x * x.transpose() / xdotx;
    (*dx_norm) /= norm_x;

    if (ddx_norm) {
      auto dx_norm_transpose = transposeGrad(*dx_norm, x.rows());
      auto ddx_norm_times_norm = -matGradMultMat(x_norm, x_norm.transpose(), (*dx_norm), dx_norm_transpose);
      auto dnorm_inv = -x.transpose() / (xdotx * norm_x);
      (*ddx_norm) = ddx_norm_times_norm / norm_x;
      auto temp = (*dx_norm) * norm_x;
      typename Derived::Index n = x.rows();
      for (int col = 0; col < n; col++) {
        auto column_as_matrix = (dnorm_inv(0, col) * temp);
        for (int row_block = 0; row_block < n; row_block++) {
          ddx_norm->block(row_block * n, col, n, 1) += column_as_matrix.col(row_block);
        }
      }
    }
  }
}

/*
 * rotation conversion gradient functions
 */
template <typename Derived>
inline typename Gradient<Eigen::Matrix<typename Derived::Scalar, 3, 3>, QUAT_SIZE>::type dquat2rotmat(const Eigen::MatrixBase<Derived>& q) {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, QUAT_SIZE);

  typename Gradient<Eigen::Matrix<typename Derived::Scalar, 3, 3>, QUAT_SIZE>::type ret;
  typename Eigen::MatrixBase<Derived>::PlainObject qtilde;
  typename Gradient<Derived, QUAT_SIZE>::type dqtilde;
  normalizeVec(q, qtilde, &dqtilde);

  typedef typename Derived::Scalar Scalar;
  Scalar w=qtilde(0);
  Scalar x=qtilde(1);
  Scalar y=qtilde(2);
  Scalar z=qtilde(3);

  ret << w, x, -y, -z, z, y, x, w, -y, z, -w, x, -z, y, x, -w, w, -x, y, -z, x, w, z, y, y, z, w, x, -x, -w, z, y, w, -x, -y, z;
  ret *= 2.0;
  ret *= dqtilde;
  return ret;
}

template <typename DerivedR, typename DerivedDR>
inline typename Gradient<Eigen::Matrix<typename DerivedR::Scalar, RPY_SIZE, 1>, DerivedDR::ColsAtCompileTime>::type drotmat2rpy(
    const Eigen::MatrixBase<DerivedR>& R,
    const Eigen::MatrixBase<DerivedDR>& dR) {
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<DerivedR>, SPACE_DIMENSION, SPACE_DIMENSION);
  EIGEN_STATIC_ASSERT(Eigen::MatrixBase<DerivedDR>::RowsAtCompileTime == RotmatSize, THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);

  typename DerivedDR::Index nq = dR.cols();
  typedef typename DerivedR::Scalar Scalar;
  typedef typename Gradient<Eigen::Matrix<Scalar, RPY_SIZE, 1>, DerivedDR::ColsAtCompileTime>::type ReturnType;
  ReturnType drpy(RPY_SIZE, nq);

  auto dR11_dq = getSubMatrixGradient<DerivedDR::ColsAtCompileTime>(dR, 0, 0, R.rows());
  auto dR21_dq = getSubMatrixGradient<DerivedDR::ColsAtCompileTime>(dR, 1, 0, R.rows());
  auto dR31_dq = getSubMatrixGradient<DerivedDR::ColsAtCompileTime>(dR, 2, 0, R.rows());
  auto dR32_dq = getSubMatrixGradient<DerivedDR::ColsAtCompileTime>(dR, 2, 1, R.rows());
  auto dR33_dq = getSubMatrixGradient<DerivedDR::ColsAtCompileTime>(dR, 2, 2, R.rows());

  Scalar sqterm = R(2,1) * R(2,1) + R(2,2) * R(2,2);

  using namespace std;
  // droll_dq
  drpy.row(0) = (R(2, 2) * dR32_dq - R(2, 1) * dR33_dq) / sqterm;

  // dpitch_dq
  Scalar sqrt_sqterm = sqrt(sqterm);
  drpy.row(1) = (-sqrt_sqterm * dR31_dq + R(2, 0) / sqrt_sqterm * (R(2, 1) * dR32_dq + R(2, 2) * dR33_dq)) / (R(2, 0) * R(2, 0) + R(2, 1) * R(2, 1) + R(2, 2) * R(2, 2));

  // dyaw_dq
  sqterm = R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0);
  drpy.row(2) = (R(0, 0) * dR21_dq - R(1, 0) * dR11_dq) / sqterm;
  return drpy;
}

template <typename DerivedR, typename DerivedDR>
inline typename Gradient<Eigen::Matrix<typename DerivedR::Scalar, QUAT_SIZE, 1>, DerivedDR::ColsAtCompileTime>::type drotmat2quat(
    const Eigen::MatrixBase<DerivedR>& R,
    const Eigen::MatrixBase<DerivedDR>& dR) {
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<DerivedR>, SPACE_DIMENSION, SPACE_DIMENSION);
  EIGEN_STATIC_ASSERT(Eigen::MatrixBase<DerivedDR>::RowsAtCompileTime == RotmatSize, THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);

  typedef typename DerivedR::Scalar Scalar;
  typedef typename Gradient<Eigen::Matrix<Scalar, QUAT_SIZE, 1>, DerivedDR::ColsAtCompileTime>::type ReturnType;
  typename DerivedDR::Index nq = dR.cols();

  auto dR11_dq = getSubMatrixGradient<DerivedDR::ColsAtCompileTime>(dR, 0, 0, R.rows());
  auto dR12_dq = getSubMatrixGradient<DerivedDR::ColsAtCompileTime>(dR, 0, 1, R.rows());
  auto dR13_dq = getSubMatrixGradient<DerivedDR::ColsAtCompileTime>(dR, 0, 2, R.rows());
  auto dR21_dq = getSubMatrixGradient<DerivedDR::ColsAtCompileTime>(dR, 1, 0, R.rows());
  auto dR22_dq = getSubMatrixGradient<DerivedDR::ColsAtCompileTime>(dR, 1, 1, R.rows());
  auto dR23_dq = getSubMatrixGradient<DerivedDR::ColsAtCompileTime>(dR, 1, 2, R.rows());
  auto dR31_dq = getSubMatrixGradient<DerivedDR::ColsAtCompileTime>(dR, 2, 0, R.rows());
  auto dR32_dq = getSubMatrixGradient<DerivedDR::ColsAtCompileTime>(dR, 2, 1, R.rows());
  auto dR33_dq = getSubMatrixGradient<DerivedDR::ColsAtCompileTime>(dR, 2, 2, R.rows());

  Eigen::Matrix<Scalar, 4, 3> A;
  A.row(0) << 1.0, 1.0, 1.0;
  A.row(1) << 1.0, -1.0, -1.0;
  A.row(2) << -1.0, 1.0, -1.0;
  A.row(3) << -1.0, -1.0, 1.0;
  Eigen::Matrix<Scalar, 4, 1> B = A * R.diagonal();
  typename Eigen::Matrix<Scalar, 4, 1>::Index ind, max_col;
  Scalar val = B.maxCoeff(&ind, &max_col);

  ReturnType dq(QUAT_SIZE, nq);
  using namespace std;
  switch (ind) {
    case 0: {
      // val = trace(M)
      auto dvaldq = dR11_dq + dR22_dq + dR33_dq;
      auto dwdq = dvaldq / (4.0 * sqrt(1.0 + val));
      auto w = sqrt(1.0 + val) / 2.0;
      auto wsquare4 = 4.0 * w * w;
      dq.row(0) = dwdq;
      dq.row(1) = ((dR32_dq - dR23_dq) * w - (R(2, 1) - R(1, 2)) * dwdq) / wsquare4;
      dq.row(2) = ((dR13_dq - dR31_dq) * w - (R(0, 2) - R(2, 0)) * dwdq) / wsquare4;
      dq.row(3) = ((dR21_dq - dR12_dq) * w - (R(1, 0) - R(0, 1)) * dwdq) / wsquare4;
      break;
    }
    case 1: {
      // val = M(1,1) - M(2,2) - M(3,3)
      auto dvaldq = dR11_dq - dR22_dq - dR33_dq;
      auto s = 2.0 * sqrt(1.0 + val);
      auto ssquare = s * s;
      auto dsdq = dvaldq / sqrt(1.0 + val);
      dq.row(0) = ((dR32_dq - dR23_dq) * s - (R(2, 1) - R(1, 2)) * dsdq) / ssquare;
      dq.row(1) = .25 * dsdq;
      dq.row(2) = ((dR12_dq + dR21_dq) * s - (R(0, 1) + R(1, 0)) * dsdq) / ssquare;
      dq.row(3) = ((dR13_dq + dR31_dq) * s - (R(0, 2) + R(2, 0)) * dsdq) / ssquare;
      break;
    }
    case 2: {
      // val = M(2,2) - M(1,1) - M(3,3)
      auto dvaldq = -dR11_dq + dR22_dq - dR33_dq;
      auto s = 2.0 * (sqrt(1.0 + val));
      auto ssquare = s * s;
      auto dsdq = dvaldq / sqrt(1.0 + val);
      dq.row(0) = ((dR13_dq - dR31_dq) * s - (R(0, 2) - R(2, 0)) * dsdq) / ssquare;
      dq.row(1) = ((dR12_dq + dR21_dq) * s - (R(0, 1) + R(1, 0)) * dsdq) / ssquare;
      dq.row(2) = .25 * dsdq;
      dq.row(3) = ((dR23_dq + dR32_dq) * s - (R(1, 2) + R(2, 1)) * dsdq) / ssquare;
      break;
    }
    default: {
      // val = M(3,3) - M(2,2) - M(1,1)
      auto dvaldq = -dR11_dq - dR22_dq + dR33_dq;
      auto s = 2.0 * (sqrt(1.0 + val));
      auto ssquare = s * s;
      auto dsdq = dvaldq / sqrt(1.0 + val);
      dq.row(0) = ((dR21_dq - dR12_dq) * s - (R(1, 0) - R(0, 1)) * dsdq) / ssquare;
      dq.row(1) = ((dR13_dq + dR31_dq) * s - (R(0, 2) + R(2, 0)) * dsdq) / ssquare;
      dq.row(2) = ((dR23_dq + dR32_dq) * s - (R(1, 2) + R(2, 1)) * dsdq) / ssquare;
      dq.row(3) = .25 * dsdq;
      break;
    }
  }
  return dq;
}

/*
 * angular velocity conversion functions
 */
template <typename DerivedQ, typename DerivedM, typename DerivedDM>
inline void angularvel2quatdotMatrix(const Eigen::MatrixBase<DerivedQ>& q,
    Eigen::MatrixBase<DerivedM>& M,
    Eigen::MatrixBase<DerivedDM>* dM = nullptr) {
  // note: not normalizing to match MATLAB implementation
  M.resize(QUAT_SIZE, SPACE_DIMENSION);
  M.row(0) << -q(1), -q(2), -q(3);
  M.row(1) << q(0), q(3), -q(2);
  M.row(2) << -q(3), q(0), q(1);
  M.row(3) << q(2), -q(1), q(0);
  M *= 0.5;

  if (dM) {
    (*dM) << 0.0, -0.5, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.5, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0,
            -0.5, 0.0, 0.0, 0.0, 0.0, 0.5, 0.5, 0.0, 0.0, 0.0, 0.0, -0.5, 0.0, 0.0, 0.0, 0.0, 0.0, -0.5, 0.0,
            0.0, -0.5, 0.0, 0.0, 0.5, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0;
  }
}

template<typename DerivedRPY, typename DerivedPhi, typename DerivedDPhi, typename DerivedDDPhi>
inline void angularvel2rpydotMatrix(const Eigen::MatrixBase<DerivedRPY>& rpy,
    typename Eigen::MatrixBase<DerivedPhi>& phi,
    typename Eigen::MatrixBase<DerivedDPhi>* dphi = nullptr,
    typename Eigen::MatrixBase<DerivedDDPhi>* ddphi = nullptr) {
  phi.resize(RPY_SIZE, SPACE_DIMENSION);

  typedef typename DerivedRPY::Scalar Scalar;
  Scalar p = rpy(1);
  Scalar y = rpy(2);

  using namespace std;
  Scalar sy = sin(y);
  Scalar cy = cos(y);
  Scalar sp = sin(p);
  Scalar cp = cos(p);
  Scalar tp = sp / cp;

  phi << cy / cp, sy / cp, 0.0, -sy, cy, 0.0, cy * tp, tp * sy, 1.0;
  if (dphi) {
    dphi->resize(phi.size(), RPY_SIZE);
    Scalar sp2 = sp * sp;
    Scalar cp2 = cp * cp;
    (*dphi) << 0.0, (cy * sp) / cp2, -sy / cp, 0.0, 0.0, -cy, 0.0, cy + (cy * sp2) / cp2, -(sp * sy) / cp, 0.0, (sp * sy) / cp2, cy / cp, 0.0, 0.0, -sy, 0.0, sy + (sp2 * sy) / cp2, (cy * sp) / cp, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    if (ddphi) {
      ddphi->resize(dphi->size(), RPY_SIZE);
      Scalar cp3 = cp2 * cp;
      (*ddphi) << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -(cy * (cp2 - 2.0)) / cp3, (sp * sy) / (sp2 - 1.0), 0.0, 0.0, 0.0, 0.0, (2.0 * cy * sp) / cp3, sy / (sp2 - 1.0), 0.0, (2.0 * sy - cp2 * sy)
                                                                                                                                                                                                                                                                                    / cp3, (cy * sp) / cp2, 0.0, 0.0, 0.0, 0.0, (2.0 * sp * sy) / cp3, cy / cp2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, (sp * sy) / (sp2 - 1.0), -cy / cp, 0.0, 0.0, sy, 0.0, sy / (sp2 - 1.0), -(cy * sp) / cp, 0.0, (cy * sp) / cp2, -sy / cp, 0.0, 0.0, -cy, 0.0, cy / cp2, -(sp * sy)
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          / cp, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    }
  }
}

template<typename Derived>
inline GradientVar<typename Derived::Scalar, Eigen::Dynamic, SPACE_DIMENSION> angularvel2RepresentationDotMatrix(
    int rotation_type, const Eigen::MatrixBase<Derived>& qrot, int gradient_order) {
  // note: gradients w.r.t. qrot
  GradientVar<typename Derived::Scalar, Eigen::Dynamic, SPACE_DIMENSION> ret(qrot.rows(), SPACE_DIMENSION, qrot.rows(), gradient_order);
  switch (rotation_type) {
    case 0:
      // done
      break;
    case 1: {
      if (gradient_order > 1) {
        angularvel2rpydotMatrix(qrot, ret.value(), &ret.gradient().value(), &ret.gradient().gradient().value());
      }
      else if (gradient_order > 0) {
        angularvel2rpydotMatrix(qrot, ret.value(), &ret.gradient().value(), (Eigen::MatrixXd*) nullptr);
      }
      else {
        angularvel2rpydotMatrix(qrot, ret.value(), (Eigen::MatrixXd*) nullptr, (Eigen::MatrixXd*) nullptr);
      }
      break;
    }
    case 2: {
      if (gradient_order > 1) {
        ret.gradient().gradient().value().setZero();
      }
      if (gradient_order > 0) {
        angularvel2quatdotMatrix(qrot, ret.value(), &ret.gradient().value());
      }
      else {
        angularvel2quatdotMatrix(qrot, ret.value(), (Eigen::MatrixXd*) nullptr);
      }
      break;
    }
    default:
      throw std::runtime_error("rotation representation type not recognized");
  }
  return ret;
}

template<typename DerivedRPY, typename DerivedE>
inline void rpydot2angularvelMatrix(const Eigen::MatrixBase<DerivedRPY>& rpy,
    Eigen::MatrixBase<DerivedE>& E,
    typename Gradient<DerivedE,RPY_SIZE,1>::type* dE=nullptr) {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<DerivedRPY>, RPY_SIZE);
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<DerivedE>, SPACE_DIMENSION,RPY_SIZE);
  typedef typename DerivedRPY::Scalar Scalar;
  Scalar p = rpy(1);
  Scalar y = rpy(2);
  Scalar sp = sin(p);
  Scalar cp = cos(p);
  Scalar sy = sin(y);
  Scalar cy = cos(y);

  E << cp*cy, -sy, 0.0, cp*sy, cy, 0.0, -sp, 0.0, 1.0;
  if(dE)
  {
    (*dE)<< 0.0, -sp*cy, -cp*sy, 0.0, -sp*sy, cp*cy, 0.0, -cp, 0.0, 0.0, 0.0, -cy, 0.0, 0.0, -sy, 0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  }
}

template <typename DerivedQ, typename DerivedM>
inline void quatdot2angularvelMatrix(const Eigen::MatrixBase<DerivedQ>& q,
    Eigen::MatrixBase<DerivedM>& M,
    typename Gradient<DerivedM, QUAT_SIZE, 1>::type* dM = nullptr) {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<DerivedQ>, QUAT_SIZE);
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<DerivedM>, SPACE_DIMENSION, QUAT_SIZE);

  typename DerivedQ::PlainObject qtilde;
  if (dM) {
    typename Gradient<DerivedQ, QUAT_SIZE>::type dqtilde;
    normalizeVec(q, qtilde, &dqtilde);
    (*dM) << 0.0, -2.0, 0.0, 0.0, 0.0, 0.0, -2.0, 0.0, 0.0, 0.0, 0.0, -2.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, -2.0, 0.0, 0.0, 0.0, 0.0, -2.0, 2.0, 0.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, -2.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0;
    (*dM) *= dqtilde;
  } else {
    normalizeVec(q, qtilde);
  }
  M << -qtilde(1), qtilde(0), -qtilde(3), qtilde(2), -qtilde(2), qtilde(3), qtilde(0), -qtilde(1), -qtilde(3), -qtilde(2), qtilde(1), qtilde(0);
  M *= 2.0;
}

template<typename Scalar >
inline void cylindrical2cartesian(const Eigen::Matrix<Scalar,3,1> &m_cylinder_axis, const Eigen::Matrix<Scalar,3,1> &m_cylinder_x_dir, const Eigen::Matrix<Scalar,3,1> &cylinder_origin, const Eigen::Matrix<Scalar,6,1> &x_cylinder, const Eigen::Matrix<Scalar,6,1> &v_cylinder, Eigen::Matrix<Scalar,6,1> &x_cartesian, Eigen::Matrix<Scalar,6,1> &v_cartesian, Eigen::Matrix<Scalar,6,6> &J, Eigen::Matrix<Scalar,6,1> &Jdotv) {
  using namespace Eigen;
  Matrix<Scalar,3,1> cylinder_axis = m_cylinder_axis/m_cylinder_axis.norm();
  Matrix<Scalar,3,1> cylinder_x_dir = m_cylinder_x_dir/m_cylinder_x_dir.norm();
  Matrix<Scalar,3,3> R_cylinder2cartesian;
  R_cylinder2cartesian.col(0) = cylinder_x_dir;
  R_cylinder2cartesian.col(1) = cylinder_axis.cross(cylinder_x_dir);
  R_cylinder2cartesian.col(2) = cylinder_axis;
  double radius = x_cylinder(0);
  double theta = x_cylinder(1);
  double c_theta = cos(theta);
  double s_theta = sin(theta);
  double height = x_cylinder(2);
  double radius_dot = v_cylinder(0);
  double theta_dot = v_cylinder(1);
  double height_dot = v_cylinder(2);
  Matrix<Scalar,3,1> x_pos_cartesian;
  x_pos_cartesian << radius*c_theta, radius*s_theta, height;
  x_pos_cartesian = R_cylinder2cartesian*x_pos_cartesian+cylinder_origin;
  Matrix<Scalar,3,1> v_pos_cartesian;
  v_pos_cartesian << radius*-s_theta*theta_dot+radius_dot*c_theta, radius*c_theta*theta_dot+radius_dot*s_theta, height_dot;
  v_pos_cartesian = R_cylinder2cartesian*v_pos_cartesian;
  Vector3d x_rpy_cylinder = x_cylinder.block(3,0,3,1);
  Matrix<Scalar,3,3> R_tangent = rpy2rotmat(x_rpy_cylinder);
  Matrix<Scalar,3,3> R_tangent2cylinder;
  Matrix<Scalar,3,3> dR_tangent2cylinder;
  Matrix<Scalar,3,3> ddR_tangent2cylinder;
  rotz(theta-M_PI/2,R_tangent2cylinder,dR_tangent2cylinder, ddR_tangent2cylinder);
  Matrix<Scalar,3,3> dR_tangent2cylinder_dtheta = dR_tangent2cylinder;
  Matrix<Scalar,3,3> R_cylinder = R_tangent2cylinder*R_tangent;
  Matrix<Scalar,3,3> R_cartesian = R_cylinder2cartesian*R_cylinder;
  Matrix<Scalar,3,1> x_rpy_cartesian = rotmat2rpy(R_cartesian);
  x_cartesian.block(0,0,3,1) = x_pos_cartesian;
  x_cartesian.block(3,0,3,1) = x_rpy_cartesian;
  v_cartesian.block(0,0,3,1) = v_pos_cartesian;
  v_cartesian.block(3,0,3,1) = theta_dot*R_cylinder2cartesian.col(2)+R_cylinder2cartesian*R_tangent2cylinder*v_cylinder.block(3,0,3,1);
  J = Matrix<Scalar,6,6>::Zero();
  J.block(0,0,3,1) << c_theta,s_theta,0;
  J.block(0,1,3,1) << radius*-s_theta,radius*c_theta,0;
  J.block(0,2,3,1) << 0,0,1;
  J.block(0,0,3,3) = R_cylinder2cartesian*J.block(0,0,3,3);
  J.block(3,1,3,1) = R_cylinder2cartesian.col(2);
  J.block(3,3,3,3) = R_cylinder2cartesian*R_tangent2cylinder;
  Matrix<Scalar,3,3> dJ1_dradius = Matrix<Scalar,3,3>::Zero();
  dJ1_dradius(0,1) = -s_theta;
  dJ1_dradius(1,1) = c_theta;
  Matrix<Scalar,3,3> dJ1_dtheta = Matrix<Scalar,3,3>::Zero();
  dJ1_dtheta(0,0) = -s_theta;
  dJ1_dtheta(0,1) = -radius*c_theta;
  dJ1_dtheta(1,0) = c_theta;
  dJ1_dtheta(1,1) = -radius*s_theta;
  Jdotv.block(0,0,3,1) = R_cylinder2cartesian*(dJ1_dradius*radius_dot+dJ1_dtheta*theta_dot)*v_cylinder.block(0,0,3,1);
  Jdotv.block(3,0,3,1) = R_cylinder2cartesian*dR_tangent2cylinder_dtheta*theta_dot*v_cylinder.block(3,0,3,1);
}

template <typename Scalar>
inline void cartesian2cylindrical(const Eigen::Matrix<Scalar,3,1> &m_cylinder_axis, const Eigen::Matrix<Scalar,3,1> &m_cylinder_x_dir, const Eigen::Matrix<Scalar,3,1> & cylinder_origin, const Eigen::Matrix<Scalar,6,1> &x_cartesian, const Eigen::Matrix<Scalar,6,1> &v_cartesian, Eigen::Matrix<Scalar,6,1> &x_cylinder, Eigen::Matrix<Scalar,6,1> &v_cylinder, Eigen::Matrix<Scalar,6,6> &J, Eigen::Matrix<Scalar,6,1> &Jdotv ) {
  using namespace Eigen;
  Matrix<Scalar,3,1> cylinder_axis = m_cylinder_axis/m_cylinder_axis.norm();
  Matrix<Scalar,3,1> cylinder_x_dir = m_cylinder_x_dir/m_cylinder_x_dir.norm();
  Matrix<Scalar,3,3> R_cylinder2cartesian;
  R_cylinder2cartesian.col(0) = cylinder_x_dir;
  R_cylinder2cartesian.col(1) = cylinder_axis.cross(cylinder_x_dir);
  R_cylinder2cartesian.col(2) = cylinder_axis;
  Matrix<Scalar,3,3> R_cartesian2cylinder = R_cylinder2cartesian.transpose();
  Matrix<Scalar,3,1> x_pos_cylinder = R_cartesian2cylinder*(x_cartesian.block(0,0,3,1)-cylinder_origin);
  Matrix<Scalar,3,1> v_pos_cylinder = R_cartesian2cylinder*v_cartesian.block(0,0,3,1);
  double radius = sqrt(pow(x_pos_cylinder(0),2)+pow(x_pos_cylinder(1),2));
  double radius_dot = (x_pos_cylinder(0)*v_pos_cylinder(0)+x_pos_cylinder(1)*v_pos_cylinder(1))/radius;
  double theta = atan2(x_pos_cylinder(1),x_pos_cylinder(0));
  double radius_square = pow(radius,2);
  double radius_cubic = pow(radius,3);
  double radius_quad = pow(radius,4);
  double theta_dot = (-x_pos_cylinder(1)*v_pos_cylinder(0)+x_pos_cylinder(0)*v_pos_cylinder(1))/radius_square;
  double height = x_pos_cylinder(2);
  double height_dot = v_pos_cylinder(2);
  x_cylinder(0) = radius;
  x_cylinder(1) = theta;
  x_cylinder(2) = height;
  v_cylinder(0) = radius_dot;
  v_cylinder(1) = theta_dot;
  v_cylinder(2) = height_dot;
  Matrix<Scalar,3,3> R_tangent2cylinder;
  Matrix<Scalar,3,3> dR_tangent2cylinder;
  Matrix<Scalar,3,3> ddR_tangent2cylinder;
  rotz(theta-M_PI/2,R_tangent2cylinder,dR_tangent2cylinder, ddR_tangent2cylinder);
  Matrix<Scalar,3,3> R_cylinder2tangent = R_tangent2cylinder.transpose();
  Vector3d x_rpy_cartesian = x_cartesian.block(3,0,3,1);
  Matrix<Scalar,3,3> R_cartesian = rpy2rotmat(x_rpy_cartesian);
  x_cylinder.block(3,0,3,1) = rotmat2rpy(R_cylinder2tangent*R_cartesian2cylinder*R_cartesian);
  J = Matrix<Scalar,6,6>::Zero();
  Matrix<Scalar,6,6> Jdot = Matrix<Scalar,6,6>::Zero();
  J(0,0) = x_pos_cylinder(0)/radius;
  J(0,1) = x_pos_cylinder(1)/radius;
  J(1,0) = -x_pos_cylinder(1)/radius_square;
  J(1,1) = x_pos_cylinder(0)/radius_square;
  J(2,2) = 1.0;
  J.block(0,0,3,3) = J.block(0,0,3,3)*R_cartesian2cylinder;
  Jdot(0,0) = pow(x_pos_cylinder(1),2)/radius_cubic*v_pos_cylinder(0)-x_pos_cylinder(0)*x_pos_cylinder(1)/radius_cubic*v_pos_cylinder(1);
  Jdot(0,1) = -x_pos_cylinder(0)*x_pos_cylinder(1)/radius_cubic*v_pos_cylinder(0)+pow(x_pos_cylinder(0),2)/radius_cubic*v_pos_cylinder(1);
  Jdot(1,0) = 2*x_pos_cylinder(0)*x_pos_cylinder(1)/radius_quad*v_pos_cylinder(0)+(pow(x_pos_cylinder(1),2)-pow(x_pos_cylinder(0),2))/radius_quad*v_pos_cylinder(1);
  Jdot(1,1) = (pow(x_pos_cylinder(1),2)-pow(x_pos_cylinder(0),2))/radius_quad*v_pos_cylinder(0)-2*x_pos_cylinder(0)*x_pos_cylinder(1)/radius_quad*v_pos_cylinder(1);
  Jdot.block(0,0,3,3) = Jdot.block(0,0,3,3)*R_cartesian2cylinder;
  v_cylinder.block(3,0,3,1) = R_cylinder2tangent*R_cartesian2cylinder*v_cartesian.block(3,0,3,1)-theta_dot*R_cylinder2tangent.col(2);
  J.block(3,0,3,3) = R_cylinder2tangent.col(2)*-J.block(1,0,1,3);
  J.block(3,3,3,3) = R_cylinder2tangent*R_cartesian2cylinder;
  Jdot.block(3,0,3,3) = dR_tangent2cylinder.row(2).transpose()*-J.block(1,0,1,3)*theta_dot+R_cylinder2tangent.col(2)*-Jdot.block(1,0,1,3);
  Jdot.block(3,3,3,3) = dR_tangent2cylinder.transpose()*theta_dot*R_cartesian2cylinder;
  Jdotv = Jdot*v_cartesian;
}

/*
 * spatial transform functions
 */
template<typename Derived>
struct TransformSpatial {
  typedef typename Eigen::Matrix<typename Derived::Scalar, TWIST_SIZE, Derived::ColsAtCompileTime> type;
};

template<typename DerivedM>
inline typename TransformSpatial<DerivedM>::type transformSpatialMotion(
    const Eigen::Transform<typename DerivedM::Scalar, 3, Eigen::Isometry>& T,
    const Eigen::MatrixBase<DerivedM>& M) {
  Eigen::Matrix<typename DerivedM::Scalar, TWIST_SIZE, DerivedM::ColsAtCompileTime> ret(TWIST_SIZE, M.cols());
  ret.template topRows<3>().noalias() = T.linear() * M.template topRows<3>();
  ret.template bottomRows<3>().noalias() = -ret.template topRows<3>().colwise().cross(T.translation());
  ret.template bottomRows<3>().noalias() += T.linear() * M.template bottomRows<3>();
  return ret;
}

template<typename DerivedF>
inline typename TransformSpatial<DerivedF>::type transformSpatialForce(
    const Eigen::Transform<typename DerivedF::Scalar, 3, Eigen::Isometry>& T,
    const Eigen::MatrixBase<DerivedF>& F) {
  Eigen::Matrix<typename DerivedF::Scalar, TWIST_SIZE, DerivedF::ColsAtCompileTime> ret(TWIST_SIZE, F.cols());
  ret.template bottomRows<3>().noalias() = T.linear() * F.template bottomRows<3>().eval();
  ret.template topRows<3>() = -ret.template bottomRows<3>().colwise().cross(T.translation());
  ret.template topRows<3>().noalias() += T.linear() * F.template topRows<3>();
  return ret;
}

template <typename Scalar, typename DerivedX, typename DerivedDT, typename DerivedDX>
inline typename Gradient<DerivedX, DerivedDX::ColsAtCompileTime>::type dTransformSpatialForce(
        const Eigen::Transform<Scalar, 3, Eigen::Isometry>& T,
        const Eigen::MatrixBase<DerivedX>& X,
        const Eigen::MatrixBase<DerivedDT>& dT,
        const Eigen::MatrixBase<DerivedDX>& dX) {
  assert(dT.cols() == dX.cols());
  typename DerivedDT::Index nq = dT.cols();

  const auto& R = T.linear();
  const auto& p = T.translation();

  std::array<int, 3> rows = {0, 1, 2};
  std::array<int, 3> R_cols = {0, 1, 2};
  std::array<int, 1> p_cols = {3};

  auto dR = getSubMatrixGradient<Eigen::Dynamic>(dT, rows, R_cols, T.Rows);
  auto dp = getSubMatrixGradient<Eigen::Dynamic>(dT, rows, p_cols, T.Rows);

  typename Gradient<DerivedX, DerivedDX::ColsAtCompileTime>::type ret(X.size(), nq);
  std::array<int, 3> Xomega_rows = {0, 1, 2};
  std::array<int, 3> Xv_rows = {3, 4, 5};
  for (int col = 0; col < X.cols(); col++) {
    auto Xomega_col = X.template block<3, 1>(0, col);
    auto Xv_col = X.template block<3, 1>(3, col);

    auto RXv_col = (R * Xv_col).eval();

    std::array<int, 1> col_array = {col};
    auto dXomega_col = getSubMatrixGradient<Eigen::Dynamic>(dX, Xomega_rows, col_array, X.rows());
    auto dXv_col = getSubMatrixGradient<Eigen::Dynamic>(dX, Xv_rows, col_array, X.rows());

    auto domega_part_col = (R * dXomega_col).eval();
    domega_part_col += matGradMult(dR, Xomega_col);
    auto dv_part_col = (R * dXv_col).eval();
    dv_part_col += matGradMult(dR, Xv_col);
    domega_part_col += dp.colwise().cross(RXv_col);
    domega_part_col -= dv_part_col.colwise().cross(p);

    setSubMatrixGradient<Eigen::Dynamic>(ret, domega_part_col, Xomega_rows, col_array, X.rows());
    setSubMatrixGradient<Eigen::Dynamic>(ret, dv_part_col, Xv_rows, col_array, X.rows());
  }
  return ret;
}

template<typename DerivedI>
inline GradientVar<typename DerivedI::Scalar, TWIST_SIZE, TWIST_SIZE> transformSpatialInertia(
    const Eigen::Transform<typename DerivedI::Scalar, SPACE_DIMENSION, Eigen::Isometry>& T_current_to_new,
    const typename Gradient<typename Eigen::Transform<typename DerivedI::Scalar, SPACE_DIMENSION, Eigen::Isometry>::MatrixType, Eigen::Dynamic>::type* dT_current_to_new,
    const Eigen::MatrixBase<DerivedI>& I) {
  int gradient_order;
  typename DerivedI::Index nq;
  if (dT_current_to_new) {
    gradient_order = 1;
    nq = dT_current_to_new->cols();
  }
  else {
    nq = 0;
    gradient_order = 0;
  }

  GradientVar<typename DerivedI::Scalar, TWIST_SIZE, TWIST_SIZE> ret(TWIST_SIZE, TWIST_SIZE, nq, gradient_order);
  auto I_half_transformed = transformSpatialForce(T_current_to_new, I);

  ret.value() = transformSpatialForce(T_current_to_new, I_half_transformed.transpose());

  if (gradient_order > 0) {
    auto dI = Eigen::Matrix<typename DerivedI::Scalar, DerivedI::SizeAtCompileTime, Eigen::Dynamic>::Zero(I.size(), nq).eval(); // TODO: would be better not to evaluate and make another explicit instantiation
    auto dI_half_transformed = dTransformSpatialForce(T_current_to_new, I, *dT_current_to_new, dI);
    auto dI_half_transformed_transpose = transposeGrad(dI_half_transformed, I_half_transformed.rows());
    ret.gradient().value() = dTransformSpatialForce(T_current_to_new, I_half_transformed.transpose(), *dT_current_to_new, dI_half_transformed_transpose);
  }
  return ret;
}

template<typename DerivedA, typename DerivedB>
inline typename TransformSpatial<DerivedB>::type crossSpatialMotion(
  const Eigen::MatrixBase<DerivedA>& a,
  const Eigen::MatrixBase<DerivedB>& b) {
  typename TransformSpatial<DerivedB>::type ret(TWIST_SIZE, b.cols());
  ret.template topRows<3>() = -b.template topRows<3>().colwise().cross(a.template topRows<3>());
  ret.template bottomRows<3>() = -b.template topRows<3>().colwise().cross(a.template bottomRows<3>());
  ret.template bottomRows<3>() -= b.template bottomRows<3>().colwise().cross(a.template topRows<3>());
  return ret;
}

template<typename DerivedA, typename DerivedB>
inline typename TransformSpatial<DerivedB>::type crossSpatialForce(
  const Eigen::MatrixBase<DerivedA>& a,
  const Eigen::MatrixBase<DerivedB>& b) {
  typename TransformSpatial<DerivedB>::type ret(TWIST_SIZE, b.cols());
  ret.template topRows<3>() = -b.template topRows<3>().colwise().cross(a.template topRows<3>());
  ret.template topRows<3>() -= b.template bottomRows<3>().colwise().cross(a.template bottomRows<3>());
  ret.template bottomRows<3>() = -b.template bottomRows<3>().colwise().cross(a.template topRows<3>());
  return ret;
}

template<typename DerivedA, typename DerivedB>
inline Eigen::Matrix<typename DerivedA::Scalar, TWIST_SIZE, Eigen::Dynamic> dCrossSpatialMotion(
  const Eigen::MatrixBase<DerivedA>& a,
  const Eigen::MatrixBase<DerivedB>& b,
  const typename Gradient<DerivedA, Eigen::Dynamic>::type& da,
  const typename Gradient<DerivedB, Eigen::Dynamic>::type& db) {
  Eigen::Matrix<typename DerivedA::Scalar, TWIST_SIZE, Eigen::Dynamic> ret(TWIST_SIZE, da.cols());
  ret.row(0) = -da.row(2)*b[1] + da.row(1)*b[2] - a[2]*db.row(1) + a[1]*db.row(2);
  ret.row(1) =  da.row(2)*b[0] - da.row(0)*b[2] + a[2]*db.row(0) - a[0]*db.row(2);
  ret.row(2) = -da.row(1)*b[0] + da.row(0)*b[1] - a[1]*db.row(0) + a[0]*db.row(1);
  ret.row(3) = -da.row(5)*b[1] + da.row(4)*b[2] - da.row(2)*b[4] + da.row(1)*b[5] - a[5]*db.row(1) + a[4]*db.row(2) - a[2]*db.row(4) + a[1]*db.row(5);
  ret.row(4) =  da.row(5)*b[0] - da.row(3)*b[2] + da.row(2)*b[3] - da.row(0)*b[5] + a[5]*db.row(0) - a[3]*db.row(2) + a[2]*db.row(3) - a[0]*db.row(5);
  ret.row(5) = -da.row(4)*b[0] + da.row(3)*b[1] - da.row(1)*b[3] + da.row(0)*b[4] - a[4]*db.row(0) + a[3]*db.row(1) - a[1]*db.row(3) + a[0]*db.row(4);
  return ret;
}

template<typename DerivedA, typename DerivedB>
inline Eigen::Matrix<typename DerivedA::Scalar, TWIST_SIZE, Eigen::Dynamic> dCrossSpatialForce(
  const Eigen::MatrixBase<DerivedA>& a,
  const Eigen::MatrixBase<DerivedB>& b,
  const typename Gradient<DerivedA, Eigen::Dynamic>::type& da,
  const typename Gradient<DerivedB, Eigen::Dynamic>::type& db) {
  Eigen::Matrix<typename DerivedA::Scalar, TWIST_SIZE, Eigen::Dynamic> ret(TWIST_SIZE, da.cols());
  ret.row(0) =  da.row(2)*b[1] - da.row(1)*b[2] + da.row(5)*b[4] - da.row(4)*b[5] + a[2]*db.row(1) - a[1]*db.row(2) + a[5]*db.row(4) - a[4]*db.row(5);
  ret.row(1) = -da.row(2)*b[0] + da.row(0)*b[2] - da.row(5)*b[3] + da.row(3)*b[5] - a[2]*db.row(0) + a[0]*db.row(2) - a[5]*db.row(3) + a[3]*db.row(5);
  ret.row(2) =  da.row(1)*b[0] - da.row(0)*b[1] + da.row(4)*b[3] - da.row(3)*b[4] + a[1]*db.row(0) - a[0]*db.row(1) + a[4]*db.row(3) - a[3]*db.row(4);
  ret.row(3) =  da.row(2)*b[4] - da.row(1)*b[5] + a[2]*db.row(4) - a[1]*db.row(5);
  ret.row(4) = -da.row(2)*b[3] + da.row(0)*b[5] - a[2]*db.row(3) + a[0]*db.row(5);
  ret.row(5) =  da.row(1)*b[3] - da.row(0)*b[4] + a[1]*db.row(3) - a[0]*db.row(4);
  ret = -ret;
  return ret;
}

/*
 * spatial transform gradient methods
 */
template<typename DerivedQdotToV>
struct DHomogTrans {
  typedef typename Eigen::Matrix<typename DerivedQdotToV::Scalar, HOMOGENEOUS_TRANSFORM_SIZE, DerivedQdotToV::ColsAtCompileTime> type;
};

template<typename DerivedS, typename DerivedQdotToV>
inline typename DHomogTrans<DerivedQdotToV>::type dHomogTrans(
    const Eigen::Transform<typename DerivedQdotToV::Scalar, 3, Eigen::Isometry>& T,
    const Eigen::MatrixBase<DerivedS>& S,
    const Eigen::MatrixBase<DerivedQdotToV>& qdot_to_v) {
  const int nq_at_compile_time = DerivedQdotToV::ColsAtCompileTime;
  typename DerivedQdotToV::Index nq = qdot_to_v.cols();
  auto qdot_to_twist = (S * qdot_to_v).eval();

  const int numel = HOMOGENEOUS_TRANSFORM_SIZE;
  Eigen::Matrix<typename DerivedQdotToV::Scalar, numel, nq_at_compile_time> ret(numel, nq);

  const auto& Rx = T.linear().col(0);
  const auto& Ry = T.linear().col(1);
  const auto& Rz = T.linear().col(2);

  const auto& qdot_to_omega_x = qdot_to_twist.row(0);
  const auto& qdot_to_omega_y = qdot_to_twist.row(1);
  const auto& qdot_to_omega_z = qdot_to_twist.row(2);

  ret.template middleRows<3>(0) = -Rz * qdot_to_omega_y + Ry * qdot_to_omega_z;
  ret.row(3).setZero();

  ret.template middleRows<3>(4) = Rz * qdot_to_omega_x - Rx * qdot_to_omega_z;
  ret.row(7).setZero();

  ret.template middleRows<3>(8) = -Ry * qdot_to_omega_x + Rx * qdot_to_omega_y;
  ret.row(11).setZero();

  ret.template middleRows<3>(12) = T.linear() * qdot_to_twist.bottomRows(3);
  ret.row(15).setZero();

  return ret;
}

template<typename DerivedDT>
inline typename DHomogTrans<DerivedDT>::type dHomogTransInv(
    const Eigen::Transform<typename DerivedDT::Scalar, 3, Eigen::Isometry>& T,
    const Eigen::MatrixBase<DerivedDT>& dT) {
  typename DerivedDT::Index nq = dT.cols();

  const auto& R = T.linear();
  const auto& p = T.translation();

  std::array<int, 3> rows = {0, 1, 2};
  std::array<int, 3> R_cols = {0, 1, 2};
  std::array<int, 1> p_cols = {3};

  auto dR = getSubMatrixGradient<Eigen::Dynamic>(dT, rows, R_cols, T.Rows);
  auto dp = getSubMatrixGradient<Eigen::Dynamic>(dT, rows, p_cols, T.Rows);

  auto dinvT_R = transposeGrad(dR, R.rows());
  auto dinvT_p = (-R.transpose() * dp - matGradMult(dinvT_R, p)).eval();

  const int numel = HOMOGENEOUS_TRANSFORM_SIZE;
  Eigen::Matrix<typename DerivedDT::Scalar, numel, DerivedDT::ColsAtCompileTime> ret(numel, nq);
  setSubMatrixGradient<Eigen::Dynamic>(ret, dinvT_R, rows, R_cols, T.Rows);
  setSubMatrixGradient<Eigen::Dynamic>(ret, dinvT_p, rows, p_cols, T.Rows);

  // zero out gradient of elements in last row:
  const int last_row = 3;
  for (int col = 0; col < T.HDim; col++) {
    ret.row(last_row + col * T.Rows).setZero();
  }

  return ret;
}

template <typename Scalar, typename DerivedX, typename DerivedDT, typename DerivedDX>
inline typename Gradient<DerivedX, DerivedDX::ColsAtCompileTime, 1>::type dTransformSpatialMotion(
    const Eigen::Transform<Scalar, 3, Eigen::Isometry>& T,
    const Eigen::MatrixBase<DerivedX>& X,
    const Eigen::MatrixBase<DerivedDT>& dT,
    const Eigen::MatrixBase<DerivedDX>& dX) {
  assert(dT.cols() == dX.cols());
  typename DerivedDT::Index nq = dT.cols();

  const auto& R = T.linear();
  const auto& p = T.translation();

  std::array<int, 3> rows = {0, 1, 2};
  std::array<int, 3> R_cols = {0, 1, 2};
  std::array<int, 1> p_cols = {3};

  auto dR = getSubMatrixGradient<Eigen::Dynamic>(dT, rows, R_cols, T.Rows);
  auto dp = getSubMatrixGradient<Eigen::Dynamic>(dT, rows, p_cols, T.Rows);

  typename Gradient<DerivedX, DerivedDX::ColsAtCompileTime, 1>::type ret(X.size(), nq);
  std::array<int, 3> Xomega_rows = {0, 1, 2};
  std::array<int, 3> Xv_rows = {3, 4, 5};
  for (int col = 0; col < X.cols(); col++) {
    auto Xomega_col = X.template block<3, 1>(0, col);
    auto Xv_col = X.template block<3, 1>(3, col);

    auto RXomega_col = (R * Xomega_col).eval();

    std::array<int, 1> col_array = {col};
    auto dXomega_col = getSubMatrixGradient<Eigen::Dynamic>(dX, Xomega_rows, col_array, X.rows());
    auto dXv_col = getSubMatrixGradient<Eigen::Dynamic>(dX, Xv_rows, col_array, X.rows());

    auto domega_part_col = (R * dXomega_col + matGradMult(dR, Xomega_col)).eval();
    auto dv_part_col = (R * dXv_col + matGradMult(dR, Xv_col)).eval();
    dv_part_col += dp.colwise().cross(RXomega_col);
    dv_part_col -= domega_part_col.colwise().cross(p);

    setSubMatrixGradient<Eigen::Dynamic>(ret, domega_part_col, Xomega_rows, col_array, X.rows());
    setSubMatrixGradient<Eigen::Dynamic>(ret, dv_part_col, Xv_rows, col_array, X.rows());
  }
  return ret;
}

inline GradientVar<double,3,1> quat2expmap(const Eigen::Ref<const Eigen::Vector4d> &q, int gradient_order) {
  double t = sqrt(1-q(0)*q(0));
  bool is_degenerate=(t*t<std::numeric_limits<double>::epsilon());
  double s = is_degenerate?2.0:2.0*acos(q(0))/t;
  GradientVar<double,3,1> ret(3,1,4,gradient_order);
  ret.value() = s*q.tail(3);
  if(gradient_order>0)
  {
    ret.gradient().value() = Eigen::Matrix<double,3,4>::Zero();
    double dsdq1 = is_degenerate?0.0: (-2*t+2*acos(q(0))*q(0))/pow(t,3);
    ret.gradient().value().col(0) = q.tail(3)*dsdq1;
    ret.gradient().value().block(0,1,3,3) = Eigen::Matrix3d::Identity()*s;
  }
  else if(gradient_order>1)
  {
    throw std::runtime_error("gradient_order>1 is not supported in quat2expmap");
  }
  return ret;
};

inline GradientVar<double,3,1> flipExpmap(const Eigen::Ref<const Eigen::Vector3d> &expmap, int gradient_order) {
  if(gradient_order>1)
  {
    throw std::runtime_error("gradient_order>1 is not supported in flipExpmap");
  }
  double expmap_norm = expmap.norm();
  bool is_degenerate=(expmap_norm<std::numeric_limits<double>::epsilon());
  GradientVar<double,3,1> ret(3,1,3,gradient_order);
  Eigen::Matrix3d eye3 = Eigen::Matrix3d::Identity();
  if(is_degenerate)
  {
    ret.value() = expmap;
    if(gradient_order>0)
    {
      ret.gradient().value() = eye3;
    }
  }
  else
  {
    ret.value() = expmap-expmap/expmap_norm*2*M_PI;
    if(gradient_order>0)
    {
      ret.gradient().value() = eye3-(expmap_norm*expmap_norm*eye3-expmap*expmap.transpose())/pow(expmap_norm,3)*2*M_PI;
    }
  }
  return ret;
};

inline GradientVar<double,3,1> unwrapExpmap(const Eigen::Ref<const Eigen::Vector3d> &expmap1, const Eigen::Ref<const Eigen::Vector3d> &expmap2, int gradient_order) {
  auto expmap2_flip = flipExpmap(expmap2,gradient_order);
  double distance1 = (expmap1-expmap2).squaredNorm();
  double distance2 = (expmap1-expmap2_flip.value()).squaredNorm();
  if(distance1>distance2)
  {
    return expmap2_flip;
  }
  else
  {
    GradientVar<double,3,1> ret(3,1,3,gradient_order);
    ret.value() = expmap2;
    if(gradient_order>0)
    {
      ret.gradient().value() = Eigen::Matrix3d::Identity();
    }
    return ret;
  }
};

inline GradientVar<double,3,1> closestExpmap(const Eigen::Ref<const Eigen::Vector3d> &expmap1, const Eigen::Ref<const Eigen::Vector3d> &expmap2, int gradient_order) {
  using namespace Eigen;
  if (gradient_order>1) {
    throw std::runtime_error("closestExpmap only supports first order gradient");
  }
  double expmap1_norm = expmap1.norm();
  double expmap2_norm = expmap2.norm();
  GradientVar<double, 3, 1> ret(3,1,3,gradient_order);
  if (expmap2_norm < std::numeric_limits<double>::epsilon()) {
    if (expmap1_norm > std::numeric_limits<double>::epsilon()) {
      Vector3d expmap1_axis = expmap1/expmap1_norm;
      int expmap1_round = static_cast<int>(expmap1_norm/(2*M_PI) + 0.5);
      ret.value() = expmap1_axis*expmap1_round*2*M_PI;
      if(ret.hasGradient()) {
        ret.gradient().value() = Matrix3d::Zero();
      }
      return ret;
    }
    else {
      ret.value() = expmap2;
      if (ret.hasGradient()) {
        ret.gradient().value() = Matrix3d::Identity();
      }
    }
  }
  else {
    Vector3d expmap2_axis = expmap2/expmap2_norm;
    Matrix3d dexpmap2_axis_dexpmap2 = (expmap2_norm*Matrix3d::Identity() - expmap2*expmap2.transpose()/expmap2_norm)/pow(expmap2_norm,2);
    double expmap2_closest_k = (expmap2_axis.transpose()*expmap1 - expmap2_norm)/(2*M_PI);
    int expmap2_closest_k1;
    int expmap2_closest_k2;
    if (expmap2_closest_k>0) {
      expmap2_closest_k1 = (int) expmap2_closest_k;
    }
    else {
      expmap2_closest_k1 = (int) expmap2_closest_k - 1;
    }
    expmap2_closest_k2 = expmap2_closest_k1 + 1;
    Vector3d expmap2_closest1 = expmap2 + 2*expmap2_closest_k1*M_PI*expmap2_axis;
    Vector3d expmap2_closest2 = expmap2 + 2*expmap2_closest_k2*M_PI*expmap2_axis;
    if ((expmap2_closest1 - expmap1).norm() < (expmap2_closest2 - expmap1).norm()) {
      ret.value() = expmap2_closest1;
      if (ret.hasGradient()) {
        ret.gradient().value() = Matrix3d::Identity() + 2*dexpmap2_axis_dexpmap2*(double)expmap2_closest_k1*M_PI;
      }
      return ret;
    }
    else {
      ret.value() = expmap2_closest2;
      if (ret.hasGradient()) {
        ret.gradient().value() = Matrix3d::Identity() + 2*dexpmap2_axis_dexpmap2*(double)expmap2_closest_k2*M_PI;
      }
      return ret;
    }
  }
  return ret;
}

inline void quat2expmapSequence(const Eigen::Ref<const Eigen::Matrix<double,4,Eigen::Dynamic>> &quat, const Eigen::Ref<const Eigen::Matrix<double,4,Eigen::Dynamic>> &quat_dot, Eigen::Ref<Eigen::Matrix<double,3,Eigen::Dynamic>> expmap, Eigen::Ref<Eigen::Matrix<double,3,Eigen::Dynamic>> expmap_dot) {
  Eigen::DenseIndex N = quat.cols();
  if(quat_dot.cols() != N)
  {
    throw std::runtime_error("quat_dot must have the same number of columns as quat in quat2expmapSequence");
  }
  expmap.resize(3,N);
  expmap_dot.resize(3,N);
  for(int i = 0;i<N;i++)
  {
    auto expmap_grad = quat2expmap(quat.col(i),1);
    expmap.col(i) = expmap_grad.value();
    expmap_dot.col(i) = expmap_grad.gradient().value()*quat_dot.col(i);
    if(i>=1)
    {
      auto closest_grad = closestExpmap(expmap.col(i-1),expmap.col(i),1);
      expmap.col(i) = closest_grad.value();
      expmap_dot.col(i) = closest_grad.gradient().value()*expmap_dot.col(i);
    }
  }
}



#endif
