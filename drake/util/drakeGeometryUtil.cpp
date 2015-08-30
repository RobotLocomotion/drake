#include "drakeGeometryUtil.h"
#include <iostream>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <Eigen/Sparse>

using namespace Eigen;


double angleDiff(double phi1, double phi2)
{
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


Vector4d quatConjugate(const Eigen::Vector4d& q)
{
  Vector4d q_conj;
  q_conj << q(0), -q(1), -q(2), -q(3);
  return q_conj;
}

Eigen::Matrix4d dquatConjugate()
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

  Matrix<double, 4, 7> dq_conj = dquatConjugate() * dq;
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
  de << 4.0 * r(0) * dr.row(0) + 4.0 * u.transpose() * rvec *u.transpose() * dr.block<3, 8>(1, 0), 4.0 * u.transpose() * rvec * rvec.transpose();
  return de;
}

double quatNorm(const Eigen::Vector4d& q)
{
  return std::acos(q(0));
}

Eigen::Vector4d slerp(const Eigen::Vector4d& q1, const Eigen::Vector4d& q2, double interpolation_parameter)
{
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

  Vector4d ret = q1 * r;
  ret += q2_sign * q2 * s;
  return ret;
}

Vector4d uniformlyRandomAxisAngle(std::default_random_engine& generator)
{
  std::normal_distribution<double> normal;
  std::uniform_real_distribution<double> uniform(-M_PI, M_PI);
  double angle = uniform(generator);
  Vector3d axis = Vector3d(normal(generator), normal(generator), normal(generator));
  axis.normalize();
  Vector4d a;
  a << axis, angle;
  return a;
}

Vector4d uniformlyRandomQuat(std::default_random_engine& generator)
{
  return axis2quat(uniformlyRandomAxisAngle(generator));
}

Eigen::Matrix3d uniformlyRandomRotmat(std::default_random_engine& generator)
{
  return axis2rotmat(uniformlyRandomAxisAngle(generator));
}

Eigen::Vector3d uniformlyRandomRPY(std::default_random_engine& generator)
{
  return axis2rpy(uniformlyRandomAxisAngle(generator));
}

DLLEXPORT int rotationRepresentationSize(int rotation_type)
{
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

Matrix3d rotz(double theta) {
  // returns 3D rotation matrix (about the z axis)
  Matrix3d M;
  double c=cos(theta);
  double s=sin(theta);
  M << c,-s, 0,
      s, c, 0,
      0, 0, 1;
  return M;
}


void rotz(double theta, Matrix3d &M, Matrix3d &dM, Matrix3d &ddM)
{
  double c=cos(theta), s=sin(theta);
  M << c,-s,0, s,c,0, 0,0,1;
  dM << -s,-c,0, c,-s,0, 0,0,0;
  ddM << -c,s,0, -s,-c,0, 0,0,0;
}

DLLEXPORT GradientVar<double,3,1> quat2expmap(const Ref<const Vector4d> &q, int gradient_order)
{
  double t = sqrt(1-q(0)*q(0));
  bool is_degenerate=(t*t<std::numeric_limits<double>::epsilon());
  double s = is_degenerate?2.0:2.0*acos(q(0))/t;
  GradientVar<double,3,1> ret(3,1,4,gradient_order);
  ret.value() = s*q.tail(3);
  if(gradient_order>0)
  {
    ret.gradient().value() = Matrix<double,3,4>::Zero();
    double dsdq1 = is_degenerate?0.0: (-2*t+2*acos(q(0))*q(0))/pow(t,3);
    ret.gradient().value().col(0) = q.tail(3)*dsdq1;
    ret.gradient().value().block(0,1,3,3) = Matrix3d::Identity()*s;
  }
  else if(gradient_order>1)
  {
    throw std::runtime_error("gradient_order>1 is not supported in quat2expmap");
  }
  return ret;
}

DLLEXPORT GradientVar<double,3,1> flipExpmap(const Ref<const Vector3d> &expmap, int gradient_order)
{
  if(gradient_order>1)
  {
    throw std::runtime_error("gradient_order>1 is not supported in flipExpmap");
  }
  double expmap_norm = expmap.norm();
  bool is_degenerate=(expmap_norm<std::numeric_limits<double>::epsilon());
  GradientVar<double,3,1> ret(3,1,3,gradient_order);
  Matrix3d eye3 = Matrix3d::Identity();
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
}

DLLEXPORT GradientVar<double, 3,1> unwrapExpmap(const Ref<const Vector3d> & expmap1, const Ref<const Vector3d> &expmap2, int gradient_order)
{
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
      ret.gradient().value() = Matrix3d::Identity();
    }
    return ret;
  }
}


void quat2expmapSequence(const Ref<const Matrix<double,4,Dynamic>> &quat, const Ref<const Matrix<double,4,Dynamic>> &quat_dot, Ref<Matrix<double,3,Dynamic>> expmap, Ref<Matrix<double,3,Dynamic>> expmap_dot)
{
  DenseIndex N = quat.cols();
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

DLLEXPORT GradientVar<double, 3,1> closestExpmap(const Ref<const Vector3d> & expmap1, const Ref<const Vector3d> &expmap2, int gradient_order)
{
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
