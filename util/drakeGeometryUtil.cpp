#include "drakeGeometryUtil.h"
#include <iostream>
#include <cmath>
#include <limits>

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

Eigen::Vector4d quatConjugate(const Eigen::Vector4d& q)
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

template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 3, 1> quat2rpy(const Eigen::MatrixBase<Derived>& q)
{
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
Eigen::Matrix<typename Derived::Scalar, 3, 3> quat2rotmat(const Eigen::MatrixBase<Derived>& q)
{
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
Eigen::Matrix<typename Derived::Scalar, 4, 1> quat2axis(const Eigen::MatrixBase<Derived>& q)
{
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 4);
  auto q_normalized = q.normalized();
  auto s = std::sqrt(1.0 - q_normalized(0) * q_normalized(0)) + std::numeric_limits<typename Derived::Scalar>::epsilon();
  Eigen::Matrix<typename Derived::Scalar, 4, 1> a;

  a << q_normalized.template tail<3>() / s, 2.0 * std::acos(q_normalized(0));
  return a;
}

template <typename Derived>
Eigen::Vector4d axis2quat(const Eigen::MatrixBase<Derived>& a)
{
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
Eigen::Matrix<typename Derived::Scalar, 3, 3> axis2rotmat(const Eigen::MatrixBase<Derived>& a)
{
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 4);
  const auto& axis = a.template head<3>();
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
Eigen::Matrix<typename Derived::Scalar, 3, 1> axis2rpy(const Eigen::MatrixBase<Derived>& a)
{
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 4);
  return quat2rpy(axis2quat(a));
}

template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 4, 1> rotmat2axis(const Eigen::MatrixBase<Derived>& R)
{
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 3, 3);

  typename Derived::Scalar theta = std::acos((R.trace() - 1.0) / 2.0);
  Vector4d a;
  if (theta > std::numeric_limits<typename Derived::Scalar>::epsilon()) {
    a << R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1), theta;
    a.head<3>() *= 1.0 / (2.0 * std::sin(theta));
  }
  else {
    a << 1.0, 0.0, 0.0, 0.0;
  }
  return a;
}

template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 4, 1> rotmat2quat(const Eigen::MatrixBase<Derived>& M)
{
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 3, 3);
  using namespace std;

  Matrix<typename Derived::Scalar, 4, 3> A;
  A.row(0) << 1.0, 1.0, 1.0;
  A.row(1.0) << 1.0, -1.0, -1.0;
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
Eigen::Matrix<typename Derived::Scalar, 3, 1> rotmat2rpy(const Eigen::MatrixBase<Derived>& R)
{
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 3, 3);
  using namespace std;

  Eigen::Matrix<typename Derived::Scalar, 3, 1> rpy;
  rpy << atan2(R(2, 1), R(2, 2)), atan2(-R(2, 0), sqrt(pow(R(2, 1), 2.0) + pow(R(2, 2), 2.0))), atan2(R(1, 0), R(0, 0));
  return rpy;
}

template<typename Derived>
Eigen::Matrix<typename Derived::Scalar, 4, 1> rpy2axis(const Eigen::MatrixBase<Derived>& rpy)
{
  return quat2axis(rpy2quat(rpy));
}

template<typename Derived>
Eigen::Matrix<typename Derived::Scalar, 4, 1> rpy2quat(const Eigen::MatrixBase<Derived>& rpy)
{
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 3);
  auto rpy_2 = (rpy / 2.0).array();
  auto s = rpy_2.sin();
  auto c = rpy_2.cos();

  Vector4d q;
  q << c(0)*c(1)*c(2) + s(0)*s(1)*s(2),
        s(0)*c(1)*c(2) - c(0)*s(1)*s(2),
        c(0)*s(1)*c(2) + s(0)*c(1)*s(2),
        c(0)*c(1)*s(2) - s(0)*s(1)*c(2);

  q /= q.norm() + std::numeric_limits<typename Derived::Scalar>::epsilon();
  return q;
}

template<typename Derived>
Eigen::Matrix<typename Derived::Scalar, 3, 3> rpy2rotmat(const Eigen::MatrixBase<Derived>& rpy)
{
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

// explicit instantiations
template Vector4d quat2axis(const MatrixBase<Vector4d>&);
template Matrix3d quat2rotmat(const MatrixBase<Vector4d>& q);
template Vector3d quat2rpy(const MatrixBase<Vector4d>&);

template Vector4d axis2quat(const MatrixBase<Vector4d>&);
template Matrix3d axis2rotmat(const MatrixBase<Vector4d>&);
template Vector3d axis2rpy(const MatrixBase<Vector4d>&);

template Vector4d rotmat2axis(const MatrixBase<Matrix3d>&);
template Vector4d rotmat2quat(const MatrixBase<Matrix3d>&);
template Vector3d rotmat2rpy(const MatrixBase<Matrix3d>&);

template Vector4d rpy2axis(const Eigen::MatrixBase<Vector3d>&);
template Vector4d rpy2quat(const Eigen::MatrixBase<Vector3d>&);
template Matrix3d rpy2rotmat(const Eigen::MatrixBase<Vector3d>&);

