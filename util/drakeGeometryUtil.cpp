#include "drakeGeometryUtil.h"
#include <iostream>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <Eigen/Sparse>
#include "expmap2quat.h"

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
Eigen::Matrix<typename Derived::Scalar, 3, 1> rotmat2rpy(const Eigen::MatrixBase<Derived>& R)
{
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 3, 3);
  using namespace std;

  Eigen::Matrix<typename Derived::Scalar, 3, 1> rpy;
  rpy << atan2(R(2, 1), R(2, 2)), atan2(-R(2, 0), sqrt(pow(R(2, 1), 2.0) + pow(R(2, 2), 2.0))), atan2(R(1, 0), R(0, 0));
  return rpy;
}

template<typename Scalar>
DLLEXPORT GradientVar<Scalar, Eigen::Dynamic, 1> rotmat2Representation(const GradientVar<Scalar, SPACE_DIMENSION, SPACE_DIMENSION>& R, int rotation_type)
{
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

template <typename Derived>
GradientVar<typename Derived::Scalar, QUAT_SIZE, 1> expmap2quat(const Eigen::MatrixBase<Derived>& v, const int gradient_order)
{
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

template<typename Derived>
Eigen::Matrix<typename Derived::Scalar,9,3> drpy2rotmat(const Eigen::MatrixBase<Derived>& rpy)
{
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

// NOTE: not reshaping second derivative to Matlab geval output format!
template <typename Derived>
void normalizeVec(
    const Eigen::MatrixBase<Derived>& x,
    typename Derived::PlainObject& x_norm,
    typename Gradient<Derived, Derived::RowsAtCompileTime, 1>::type* dx_norm,
    typename Gradient<Derived, Derived::RowsAtCompileTime, 2>::type* ddx_norm) {

  typename Derived::Scalar xdotx = x.squaredNorm();
  typename Derived::Scalar norm_x = std::sqrt(xdotx);
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



template <typename Derived>
typename Gradient<Matrix<typename Derived::Scalar, 3, 3>, QUAT_SIZE>::type dquat2rotmat(const Eigen::MatrixBase<Derived>& q)
{
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, QUAT_SIZE);

  typename Gradient<Matrix<typename Derived::Scalar, 3, 3>, QUAT_SIZE>::type ret;
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
typename Gradient<Eigen::Matrix<typename DerivedR::Scalar, RPY_SIZE, 1>, DerivedDR::ColsAtCompileTime>::type drotmat2rpy(
    const Eigen::MatrixBase<DerivedR>& R,
    const Eigen::MatrixBase<DerivedDR>& dR)
{
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
typename Gradient<Eigen::Matrix<typename DerivedR::Scalar, QUAT_SIZE, 1>, DerivedDR::ColsAtCompileTime>::type drotmat2quat(
    const Eigen::MatrixBase<DerivedR>& R,
    const Eigen::MatrixBase<DerivedDR>& dR)
{
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

  Matrix<Scalar, 4, 3> A;
  A.row(0) << 1.0, 1.0, 1.0;
  A.row(1) << 1.0, -1.0, -1.0;
  A.row(2) << -1.0, 1.0, -1.0;
  A.row(3) << -1.0, -1.0, 1.0;
  Matrix<Scalar, 4, 1> B = A * R.diagonal();
  typename Matrix<Scalar, 4, 1>::Index ind, max_col;
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

template<typename Derived>
Eigen::Matrix<typename Derived::Scalar, 3, 3> vectorToSkewSymmetric(const Eigen::MatrixBase<Derived>& p)
{
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, SPACE_DIMENSION);
  Eigen::Matrix<typename Derived::Scalar, 3, 3> ret;
  ret << 0.0, -p(2), p(1), p(2), 0.0, -p(0), -p(1), p(0), 0.0;
  return ret;
}

template <typename DerivedA, typename DerivedB>
Eigen::Matrix<typename DerivedA::Scalar, 3, Eigen::Dynamic> dcrossProduct(
    const Eigen::MatrixBase<DerivedA>& a,
    const Eigen::MatrixBase<DerivedB>& b,
    const typename Gradient<DerivedA, Eigen::Dynamic>::type& da,
    const typename Gradient<DerivedB, Eigen::Dynamic>::type& db)
{
  Eigen::Matrix<typename DerivedA::Scalar, 3, Eigen::Dynamic> ret(3, da.cols());
  ret.noalias() = da.colwise().cross(b);
  ret.noalias() -= db.colwise().cross(a);
  return ret;
}

template <typename DerivedQ, typename DerivedM, typename DerivedDM>
void angularvel2quatdotMatrix(const Eigen::MatrixBase<DerivedQ>& q,
    Eigen::MatrixBase<DerivedM>& M,
    Eigen::MatrixBase<DerivedDM>* dM)
{
  // note: not normalizing to match MATLAB implementation
  M.resize(QUAT_SIZE, SPACE_DIMENSION);
  M.row(0) << -q(1), -q(2), -q(3);
  M.row(1) << q(0), q(3), -q(2);
  M.row(2) << -q(3), q(0), q(1);
  M.row(3) << q(2), -q(1), q(0);
  M *= 0.5;

  if (dM) {
    (*dM) << 0.0, -0.5, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.5, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, -0.5, 0.0, 0.0, 0.0, 0.0, 0.5, 0.5, 0.0, 0.0, 0.0, 0.0, -0.5, 0.0, 0.0, 0.0, 0.0, 0.0, -0.5, 0.0, 0.0, -0.5, 0.0, 0.0, 0.5, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0;
  }
}

template<typename DerivedQ, typename DerivedM>
void quatdot2angularvelMatrix(const Eigen::MatrixBase<DerivedQ>& q, Eigen::MatrixBase<DerivedM>& M, typename Gradient<DerivedM, QUAT_SIZE, 1>::type* dM)
{
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

template<typename DerivedRPY, typename DerivedPhi, typename DerivedDPhi, typename DerivedDDPhi>
void angularvel2rpydotMatrix(const Eigen::MatrixBase<DerivedRPY>& rpy,
    typename Eigen::MatrixBase<DerivedPhi>& phi,
    typename Eigen::MatrixBase<DerivedDPhi>* dphi,
    typename Eigen::MatrixBase<DerivedDDPhi>* ddphi)
{
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
DLLEXPORT GradientVar<typename Derived::Scalar, Eigen::Dynamic, SPACE_DIMENSION> angularvel2RepresentationDotMatrix(
    int rotation_type, const Eigen::MatrixBase<Derived>& qrot, int gradient_order)
{
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
      angularvel2rpydotMatrix(qrot, ret.value(), &ret.gradient().value(), (MatrixXd*) nullptr);
    }
    else {
      angularvel2rpydotMatrix(qrot, ret.value(), (MatrixXd*) nullptr, (MatrixXd*) nullptr);
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
      angularvel2quatdotMatrix(qrot, ret.value(), (MatrixXd*) nullptr);
    }
    break;
  }
  default:
    throw std::runtime_error("rotation representation type not recognized");
  }
  return ret;
}

template<typename DerivedRPY, typename DerivedE>
void rpydot2angularvelMatrix(const Eigen::MatrixBase<DerivedRPY>& rpy,
		Eigen::MatrixBase<DerivedE>& E,
		typename Gradient<DerivedE,RPY_SIZE,1>::type* dE)
{
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<DerivedRPY>, RPY_SIZE);
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<DerivedE>, SPACE_DIMENSION,RPY_SIZE);
  typedef typename DerivedRPY::Scalar Scalar;
  Scalar p = rpy(1);
  Scalar y = rpy(2);
  Scalar sp = sin(p);
  Scalar cp = cos(p);
  Scalar sy = sin(y);
  Scalar cy = cos(y);

  using namespace std;
  E << cp*cy, -sy, 0.0, cp*sy, cy, 0.0, -sp, 0.0, 1.0;
  if(dE)
  {
    (*dE)<< 0.0, -sp*cy, -cp*sy, 0.0, -sp*sy, cp*cy, 0.0, -cp, 0.0, 0.0, 0.0, -cy, 0.0, 0.0, -sy, 0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  }
}

template<typename DerivedM>
typename TransformSpatial<DerivedM>::type transformSpatialMotion(
    const Eigen::Transform<typename DerivedM::Scalar, 3, Eigen::Isometry>& T,
    const Eigen::MatrixBase<DerivedM>& M) {
  Eigen::Matrix<typename DerivedM::Scalar, TWIST_SIZE, DerivedM::ColsAtCompileTime> ret(TWIST_SIZE, M.cols());
  ret.template topRows<3>().noalias() = T.linear() * M.template topRows<3>();
  ret.template bottomRows<3>().noalias() = -ret.template topRows<3>().colwise().cross(T.translation());
  ret.template bottomRows<3>().noalias() += T.linear() * M.template bottomRows<3>();
  return ret;
}

template<typename DerivedF>
typename TransformSpatial<DerivedF>::type transformSpatialForce(
    const Eigen::Transform<typename DerivedF::Scalar, 3, Eigen::Isometry>& T,
    const Eigen::MatrixBase<DerivedF>& F) {
  Eigen::Matrix<typename DerivedF::Scalar, TWIST_SIZE, DerivedF::ColsAtCompileTime> ret(TWIST_SIZE, F.cols());
  ret.template bottomRows<3>().noalias() = T.linear() * F.template bottomRows<3>().eval();
  ret.template topRows<3>() = -ret.template bottomRows<3>().colwise().cross(T.translation());
  ret.template topRows<3>().noalias() += T.linear() * F.template topRows<3>();
  return ret;
}

template<typename DerivedI>
GradientVar<typename DerivedI::Scalar, TWIST_SIZE, TWIST_SIZE> transformSpatialInertia(
    const Eigen::Transform<typename DerivedI::Scalar, SPACE_DIMENSION, Eigen::Isometry>& T_current_to_new,
    const typename Gradient<typename Eigen::Transform<typename DerivedI::Scalar, SPACE_DIMENSION, Eigen::Isometry>::MatrixType, Eigen::Dynamic>::type* dT_current_to_new,
    const Eigen::MatrixBase<DerivedI>& I)
{
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
typename TransformSpatial<DerivedB>::type crossSpatialMotion(
  const Eigen::MatrixBase<DerivedA>& a,
  const Eigen::MatrixBase<DerivedB>& b) {
  typename TransformSpatial<DerivedB>::type ret(TWIST_SIZE, b.cols());
  ret.template topRows<3>() = -b.template topRows<3>().colwise().cross(a.template topRows<3>());
  ret.template bottomRows<3>() = -b.template topRows<3>().colwise().cross(a.template bottomRows<3>());
  ret.template bottomRows<3>() -= b.template bottomRows<3>().colwise().cross(a.template topRows<3>());
  return ret;
}

template<typename DerivedA, typename DerivedB>
typename TransformSpatial<DerivedB>::type crossSpatialForce(
  const Eigen::MatrixBase<DerivedA>& a,
  const Eigen::MatrixBase<DerivedB>& b) {
  typename TransformSpatial<DerivedB>::type ret(TWIST_SIZE, b.cols());
  ret.template topRows<3>() = -b.template topRows<3>().colwise().cross(a.template topRows<3>());
  ret.template topRows<3>() -= b.template bottomRows<3>().colwise().cross(a.template bottomRows<3>());
  ret.template bottomRows<3>() = -b.template bottomRows<3>().colwise().cross(a.template topRows<3>());
  return ret;
}

template<typename DerivedA, typename DerivedB>
Eigen::Matrix<typename DerivedA::Scalar, TWIST_SIZE, Eigen::Dynamic> dCrossSpatialMotion(
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
Eigen::Matrix<typename DerivedA::Scalar, TWIST_SIZE, Eigen::Dynamic> dCrossSpatialForce(
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

template<typename DerivedS, typename DerivedQdotToV>
typename DHomogTrans<DerivedQdotToV>::type dHomogTrans(
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
typename DHomogTrans<DerivedDT>::type dHomogTransInv(
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
typename Gradient<DerivedX, DerivedDX::ColsAtCompileTime, 1>::type dTransformSpatialMotion(
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

template <typename Scalar, typename DerivedX, typename DerivedDT, typename DerivedDX>
typename Gradient<DerivedX, DerivedDX::ColsAtCompileTime>::type dTransformSpatialForce(
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

template<typename Scalar >
DLLEXPORT void cylindrical2cartesian(const Matrix<Scalar,3,1> &m_cylinder_axis, const Matrix<Scalar,3,1> &m_cylinder_x_dir, const Matrix<Scalar,3,1> &cylinder_origin, const Matrix<Scalar,6,1> &x_cylinder, const Matrix<Scalar,6,1> &v_cylinder, Matrix<Scalar,6,1> &x_cartesian, Matrix<Scalar,6,1> &v_cartesian, Matrix<Scalar,6,6> &J, Matrix<Scalar,6,1> &Jdotv)
{
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
DLLEXPORT  void cartesian2cylindrical(const Eigen::Matrix<Scalar,3,1> &m_cylinder_axis, const Eigen::Matrix<Scalar,3,1> &m_cylinder_x_dir, const Eigen::Matrix<Scalar,3,1> & cylinder_origin, const Eigen::Matrix<Scalar,6,1> &x_cartesian, const Eigen::Matrix<Scalar,6,1> &v_cartesian, Eigen::Matrix<Scalar,6,1> &x_cylinder, Eigen::Matrix<Scalar,6,1> &v_cylinder, Eigen::Matrix<Scalar,6,6> &J, Eigen::Matrix<Scalar,6,1> &Jdotv )
{
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

template <typename Derived>
DLLEXPORT GradientVar<typename Derived::Scalar,3,1> quat2expmap(const MatrixBase<Derived> &q, int gradient_order)
{
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(MatrixBase<Derived>,4);
  double t = sqrt(1-q(0)*q(0));
  bool is_degenerate=(t*t<std::numeric_limits<typename Derived::Scalar>::epsilon());
  double s = is_degenerate?2.0:2.0*acos(q(0))/t;
  GradientVar<typename Derived::Scalar,3,1> ret(3,1,4,gradient_order);
  ret.value() = s*q.tail(3);
  if(gradient_order>0)
  {
    ret.gradient().value() = Matrix<typename Derived::Scalar,3,4>::Zero();
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


// explicit instantiations
template DLLEXPORT void normalizeVec(
    const MatrixBase< Vector3d >& x,
    Vector3d& x_norm,
    Gradient<Vector3d, 3, 1>::type*,
    Gradient<Vector3d, 3, 2>::type*);

template DLLEXPORT void normalizeVec(
    const MatrixBase< Vector4d >& x,
    Vector4d& x_norm,
    Gradient<Vector4d, 4, 1>::type*,
    Gradient<Vector4d, 4, 2>::type*);

template DLLEXPORT void normalizeVec(
    const MatrixBase< Map<Vector4d> >& x,
    Vector4d& x_norm,
    Gradient<Vector4d, 4, 1>::type*,
    Gradient<Vector4d, 4, 2>::type*);

template DLLEXPORT void normalizeVec(
    const MatrixBase< Eigen::Block<Eigen::Ref<Eigen::Matrix<double, -1, 1, 0, -1, 1> const, 0, Eigen::InnerStride<1> > const, 4, 1, false> >& x,
    Vector4d& x_norm,
    Gradient<Vector4d, 4, 1>::type*,
    Gradient<Vector4d, 4, 2>::type*);

template DLLEXPORT Vector4d quat2axis(const MatrixBase<Vector4d>&);
template DLLEXPORT Matrix3d quat2rotmat(const MatrixBase<Vector4d>& q);
template DLLEXPORT Matrix3d quat2rotmat(const MatrixBase<Eigen::Block<Eigen::Ref<Eigen::Matrix<double, -1, 1, 0, -1, 1> const, 0, Eigen::InnerStride<1> > const, 4, 1, false> >& q);
template DLLEXPORT Vector3d quat2rpy(const MatrixBase<Vector4d>&);

template DLLEXPORT Vector4d axis2quat(const MatrixBase<Vector4d>&);
template DLLEXPORT Matrix3d axis2rotmat(const MatrixBase<Vector4d>&);
template DLLEXPORT Vector3d axis2rpy(const MatrixBase<Vector4d>&);

template DLLEXPORT Vector4d rotmat2axis(const MatrixBase<Matrix3d>&);
template DLLEXPORT Vector4d rotmat2quat(const MatrixBase<Matrix3d>&);
template DLLEXPORT Vector3d rotmat2rpy(const MatrixBase<Matrix3d>&);
template DLLEXPORT GradientVar<double, Eigen::Dynamic, 1> rotmat2Representation(
    const GradientVar<double, SPACE_DIMENSION, SPACE_DIMENSION>& R,
    int rotation_type);

template DLLEXPORT GradientVar<double, QUAT_SIZE, 1> expmap2quat(const MatrixBase<Vector3d>& v, const int gradient_order);
template DLLEXPORT GradientVar<double, QUAT_SIZE, 1> expmap2quat(const MatrixBase<Map<Vector3d>>& v, const int gradient_order);

template DLLEXPORT Vector4d rpy2axis(const Eigen::MatrixBase<Vector3d>&);
template DLLEXPORT Vector4d rpy2quat(const Eigen::MatrixBase<Vector3d>&);
template DLLEXPORT Matrix3d rpy2rotmat(const Eigen::MatrixBase<Vector3d>&);
template DLLEXPORT Matrix3d rpy2rotmat(const Eigen::MatrixBase<Eigen::Block<Eigen::Ref<Eigen::Matrix<double, -1, 1, 0, -1, 1> const, 0, Eigen::InnerStride<1> > const, 3, 1, false>>&);

template DLLEXPORT Matrix<double,9,3> drpy2rotmat(const Eigen::MatrixBase<Vector3d>&);
template DLLEXPORT Matrix<double,9,3> drpy2rotmat(const Eigen::MatrixBase<Eigen::Block<Eigen::Ref<Eigen::Matrix<double, -1, 1, 0, -1, 1> const, 0, Eigen::InnerStride<1> > const, 3, 1, false>>&);

template DLLEXPORT Vector4d quat2axis(const MatrixBase< Map<Vector4d> >&);
template DLLEXPORT Matrix3d quat2rotmat(const MatrixBase< Map<Vector4d> >& q);
template DLLEXPORT Vector3d quat2rpy(const MatrixBase< Map<Vector4d> >&);

template DLLEXPORT Vector4d axis2quat(const MatrixBase< Map<Vector4d> >&);
template DLLEXPORT Matrix3d axis2rotmat(const MatrixBase< Map<Vector4d> >&);
template DLLEXPORT Vector3d axis2rpy(const MatrixBase< Map<Vector4d> >&);

template DLLEXPORT Vector4d rotmat2axis(const MatrixBase< Map<Matrix3d> >&);
template DLLEXPORT Vector4d rotmat2quat(const MatrixBase< Map<Matrix3d> >&);
template DLLEXPORT Vector3d rotmat2rpy(const MatrixBase< Map<Matrix3d> >&);

template DLLEXPORT Vector4d rpy2axis(const Eigen::MatrixBase< Map<Vector3d> >&);
template DLLEXPORT Vector4d rpy2quat(const Eigen::MatrixBase< Map<Vector3d> >&);
template DLLEXPORT Matrix3d rpy2rotmat(const Eigen::MatrixBase< Map<Vector3d> >&);
template DLLEXPORT Matrix<double,9,3> drpy2rotmat(const Eigen::MatrixBase< Map<Vector3d> >&);


template DLLEXPORT Eigen::Matrix<double, TWIST_SIZE, Eigen::Dynamic> transformSpatialMotion(
    const Eigen::Isometry3d&,
    const Eigen::MatrixBase< Eigen::Matrix<double, TWIST_SIZE, Eigen::Dynamic> >&);

template DLLEXPORT Eigen::Matrix<double, TWIST_SIZE, 1> transformSpatialMotion(
    const Eigen::Isometry3d&,
    const Eigen::MatrixBase< Eigen::Matrix<double, TWIST_SIZE, 1> >&);

template DLLEXPORT TransformSpatial< MatrixXd >::type transformSpatialMotion<MatrixXd>(
    const Eigen::Isometry3d&,
    const Eigen::MatrixBase< MatrixXd >&);

template DLLEXPORT TransformSpatial<Eigen::Block<Eigen::Matrix<double, -1, -1, 0, -1, -1>, 6, -1, false> >::type transformSpatialMotion(
    const Eigen::Isometry3d& T,
    const Eigen::MatrixBase<Eigen::Block<Eigen::Matrix<double, -1, -1, 0, -1, -1>, 6, -1, false> >& M);

template DLLEXPORT TransformSpatial< Matrix<double, TWIST_SIZE, Eigen::Dynamic> >::type transformSpatialForce<Matrix<double, TWIST_SIZE, Eigen::Dynamic>>(
    const Eigen::Isometry3d&,
    const Eigen::MatrixBase< Matrix<double, TWIST_SIZE, Eigen::Dynamic> >&);

template DLLEXPORT TransformSpatial< MatrixXd >::type transformSpatialForce<MatrixXd>(
    const Eigen::Isometry3d&,
    const Eigen::MatrixBase< MatrixXd >&);

template DLLEXPORT TransformSpatial<Eigen::Block<Eigen::Matrix<double, 6, -1, 0, 6, -1>, 6, -1, true>>::type transformSpatialForce(
    const Eigen::Isometry3d&,
    const Eigen::MatrixBase<Eigen::Block<Eigen::Matrix<double, 6, -1, 0, 6, -1>, 6, -1, true> >&);

template DLLEXPORT TransformSpatial<Eigen::Matrix<double, 6, 1, 0, 6, 1> >::type transformSpatialForce(
    const Eigen::Isometry3d&,
    const Eigen::MatrixBase<Eigen::Matrix<double, 6, 1, 0, 6, 1> >&);

template DLLEXPORT TransformSpatial<Eigen::Block<Eigen::Matrix<double, 6, -1, 0, 6, -1> const, 6, 1, true> >::type transformSpatialForce(
    const Eigen::Transform<Eigen::Block<Eigen::Matrix<double, 6, -1, 0, 6, -1> const, 6, 1, true>::Scalar, 3, 1, 0> &,
    const Eigen::MatrixBase<Eigen::Block<Eigen::Matrix<double, 6, -1, 0, 6, -1> const, 6, 1, true> > &);

template DLLEXPORT TransformSpatial<Map<Matrix<double, 6, 1, 0, 6, 1> const, 0, Stride<0, 0> > >::type transformSpatialForce<Map<Matrix<double, 6, 1, 0, 6, 1> const, 0, Stride<0, 0> >>(
    const Eigen::Isometry3d&,
    const Eigen::MatrixBase<Map<Matrix<double, 6, 1, 0, 6, 1> const, 0, Stride<0, 0> > >&);

template DLLEXPORT GradientVar<double, TWIST_SIZE, TWIST_SIZE> transformSpatialInertia(
    const Eigen::Transform<double, SPACE_DIMENSION, Eigen::Isometry>& T_current_to_new,
    const Gradient<Eigen::Transform<double, SPACE_DIMENSION, Eigen::Isometry>::MatrixType, Eigen::Dynamic>::type* dT_current_to_new,
    const Eigen::MatrixBase< Eigen::Matrix<double, TWIST_SIZE, TWIST_SIZE> >& I);

template DLLEXPORT GradientVar<double, TWIST_SIZE, TWIST_SIZE> transformSpatialInertia(
    const Eigen::Transform<double, SPACE_DIMENSION, Eigen::Isometry>& T_current_to_new,
    const Gradient<Eigen::Transform<double, SPACE_DIMENSION, Eigen::Isometry>::MatrixType, Eigen::Dynamic>::type* dT_current_to_new,
    const Eigen::MatrixBase< Eigen::MatrixXd >& I);

template DLLEXPORT Eigen::Matrix<double, TWIST_SIZE, Eigen::Dynamic> dCrossSpatialMotion(
  const Eigen::MatrixBase<Eigen::Matrix<double, 6, 1, 0, 6, 1> >& a,
  const Eigen::MatrixBase<Eigen::Matrix<double, 6, 1, 0, 6, 1> >& b,
  const Gradient<Eigen::Matrix<double, 6, 1, 0, 6, 1>, Eigen::Dynamic>::type& da,
  const Gradient<Eigen::Matrix<double, 6, 1, 0, 6, 1>, Eigen::Dynamic>::type& db);

template DLLEXPORT Eigen::Matrix<double, TWIST_SIZE, Eigen::Dynamic> dCrossSpatialForce(
  const Eigen::MatrixBase<Eigen::Matrix<double, 6, 1, 0, 6, 1> >& a,
  const Eigen::MatrixBase<Eigen::Matrix<double, 6, 1, 0, 6, 1> >& b,
  const Gradient<Eigen::Matrix<double, 6, 1, 0, 6, 1>, Eigen::Dynamic>::type& da,
  const Gradient<Eigen::Matrix<double, 6, 1, 0, 6, 1>, Eigen::Dynamic>::type& db);

template DLLEXPORT Gradient<Matrix3d, QUAT_SIZE>::type dquat2rotmat(const Eigen::MatrixBase<Vector4d>&);
template DLLEXPORT Gradient<Matrix3d, QUAT_SIZE>::type dquat2rotmat(const Eigen::MatrixBase< Map<Vector4d> >&);
template DLLEXPORT Gradient<Matrix3d, QUAT_SIZE>::type dquat2rotmat(const Eigen::MatrixBase<Eigen::Block<Eigen::Ref<Eigen::Matrix<double, -1, 1, 0, -1, 1> const, 0, Eigen::InnerStride<1> > const, 4, 1, false> >&);

template DLLEXPORT Gradient<Vector3d, Dynamic>::type drotmat2rpy(
    const Eigen::MatrixBase<Matrix3d>&,
    const Eigen::MatrixBase< Matrix<double, RotmatSize, Dynamic> >&);

template DLLEXPORT Gradient<Vector3d, 6>::type drotmat2rpy(
    const Eigen::MatrixBase<Matrix3d>&,
    const Eigen::MatrixBase< Matrix<double, RotmatSize, 6> >&);

template DLLEXPORT Gradient<Vector4d, Dynamic>::type drotmat2quat(
    const Eigen::MatrixBase<Matrix3d>&,
    const Eigen::MatrixBase< Matrix<double, RotmatSize, Dynamic> >&);

template DLLEXPORT
Eigen::Matrix<double, 3, 3> vectorToSkewSymmetric(const Eigen::MatrixBase<Eigen::Vector3d>&);

template DLLEXPORT Eigen::Matrix<double, 3, Eigen::Dynamic> dcrossProduct(
    const Eigen::MatrixBase<Vector3d>& a,
    const Eigen::MatrixBase<Vector3d>& b,
    const Gradient<Vector3d, Eigen::Dynamic>::type& da,
    const Gradient<Vector3d, Eigen::Dynamic>::type& db);

template DLLEXPORT Eigen::Matrix<double, 3, Eigen::Dynamic> dcrossProduct(
    const Eigen::MatrixBase<Eigen::Block<Eigen::Matrix<double, 3, -1, 0, 3, -1>, 3, 1, true>>& a,
    const Eigen::MatrixBase<Eigen::Block<Eigen::Matrix<double, -1, 1, 0, -1, 1> const, 3, 1, false>>& b,
    const Gradient<Eigen::Block<Eigen::Matrix<double, 3, -1, 0, 3, -1>, 3, 1, true>, Eigen::Dynamic>::type& da,
    const Gradient<Eigen::Block<Eigen::Matrix<double, -1, 1, 0, -1, 1> const, 3, 1, false>, Eigen::Dynamic>::type& db);

template DLLEXPORT Eigen::Matrix<double, 3, Eigen::Dynamic> dcrossProduct(
    const Eigen::MatrixBase< Eigen::Block<Eigen::Matrix<double, 3, -1, 0, 3, -1>, 3, 1, true> >& a,
    const Eigen::MatrixBase< Eigen::Block<Eigen::Matrix<double, -1, -1, 0, -1, -1> const, 3, 1, false> >& b,
    const Gradient< Eigen::Block<Eigen::Matrix<double, 3, -1, 0, 3, -1>, 3, 1, true>, Eigen::Dynamic>::type& da,
    const Gradient< Eigen::Block<Eigen::Matrix<double, -1, -1, 0, -1, -1> const, 3, 1, false>, Eigen::Dynamic>::type& db);

template DLLEXPORT Eigen::Matrix<double, 3, Eigen::Dynamic> dcrossProduct(
    const Eigen::MatrixBase<Eigen::Block<Eigen::Matrix<double, 6, 1, 0, 6, 1> const, 3, 1, false>>& a,
    const Eigen::MatrixBase<Eigen::Block<Eigen::Block<Eigen::Matrix<double, -1, -1, 0, -1, -1>, 3, -1, false>, 3, 1, true>>& b,
    const Gradient<Eigen::Block<Eigen::Matrix<double, 6, 1, 0, 6, 1> const, 3, 1, false>, Eigen::Dynamic>::type& da,
    const Gradient<Eigen::Block<Eigen::Block<Eigen::Matrix<double, -1, -1, 0, -1, -1>, 3, -1, false>, 3, 1, true>, Eigen::Dynamic>::type& db);

template DLLEXPORT Eigen::Matrix<double, 3, Eigen::Dynamic> dcrossProduct(
    const Eigen::MatrixBase<Eigen::Block<Eigen::Matrix<double, 6, 1, 0, 6, 1> const, 3, 1, false>>& a,
    const Eigen::MatrixBase<Eigen::Block<Eigen::Matrix<double, 3, -1, 0, 3, -1>, 3, 1, true>>& b,
    const Gradient<Eigen::Block<Eigen::Matrix<double, 6, 1, 0, 6, 1> const, 3, 1, false>, Eigen::Dynamic>::type& da,
    const Gradient<Eigen::Block<Eigen::Matrix<double, 3, -1, 0, 3, -1>, 3, 1, true>, Eigen::Dynamic>::type& db);

template DLLEXPORT Eigen::Matrix<double, 3, Eigen::Dynamic> dcrossProduct(
    const Eigen::MatrixBase<Eigen::Block<Eigen::Block<Eigen::Matrix<double, 6, -1, 0, 6, -1>, 3, -1, false>, 3, 1, true>>& a,
    const Eigen::MatrixBase<Eigen::Matrix<double, 3, 1, 0, 3, 1>>& b,
    const Gradient<Eigen::Block<Eigen::Block<Eigen::Matrix<double, 6, -1, 0, 6, -1>, 3, -1, false>, 3, 1, true>, Eigen::Dynamic>::type& da,
    const Gradient<Eigen::Matrix<double, 3, 1, 0, 3, 1>, Eigen::Dynamic>::type& db);

template DLLEXPORT Eigen::Matrix<double, 3, Eigen::Dynamic> dcrossProduct(
    const Eigen::MatrixBase<Eigen::Block<Eigen::Matrix<double, 6, 1, 0, 6, 1>, 3, 1, false>>& a,
    const Eigen::MatrixBase<Eigen::Matrix<double, 3, 1, 0, 3, 1>>& b,
    const Gradient<Eigen::Block<Eigen::Matrix<double, 6, 1, 0, 6, 1>, 3, 1, false>, Eigen::Dynamic>::type& da,
    const Gradient<Eigen::Matrix<double, 3, 1, 0, 3, 1>, Eigen::Dynamic>::type& db);

template DLLEXPORT Eigen::Matrix<double, 3, -1, 0, 3, -1> dcrossProduct<Eigen::Block<Eigen::Matrix<double, 6, 1, 0, 6, 1> const, 3, 1, false>, Eigen::Block<Eigen::Block<Eigen::Matrix<double, -1, 1, 0, -1, 1>, 3, 1, false>, 3, 1, true> >(Eigen::MatrixBase<Eigen::Block<Eigen::Matrix<double, 6, 1, 0, 6, 1> const, 3, 1, false> > const&, Eigen::MatrixBase<Eigen::Block<Eigen::Block<Eigen::Matrix<double, -1, 1, 0, -1, 1>, 3, 1, false>, 3, 1, true> > const&, Gradient<Eigen::Block<Eigen::Matrix<double, 6, 1, 0, 6, 1> const, 3, 1, false>, -1, 1>::type const&, Gradient<Eigen::Block<Eigen::Block<Eigen::Matrix<double, -1, 1, 0, -1, 1>, 3, 1, false>, 3, 1, true>, -1, 1>::type const&);

template DLLEXPORT Eigen::Matrix<double, 3, -1, 0, 3, -1> dcrossProduct<Eigen::Block<Eigen::Matrix<double, 6, 1, 0, 6, 1> const, 3, 1, false>, Eigen::Block<Eigen::Matrix<double, 3, 1, 0, 3, 1>, 3, 1, true> >(Eigen::MatrixBase<Eigen::Block<Eigen::Matrix<double, 6, 1, 0, 6, 1> const, 3, 1, false> > const&, Eigen::MatrixBase<Eigen::Block<Eigen::Matrix<double, 3, 1, 0, 3, 1>, 3, 1, true> > const&, Gradient<Eigen::Block<Eigen::Matrix<double, 6, 1, 0, 6, 1> const, 3, 1, false>, -1, 1>::type const&, Gradient<Eigen::Block<Eigen::Matrix<double, 3, 1, 0, 3, 1>, 3, 1, true>, -1, 1>::type const&);

template DLLEXPORT DHomogTrans<MatrixXd>::type dHomogTrans(
    const Isometry3d&,
    const MatrixBase< Matrix<double, TWIST_SIZE, Dynamic> >&,
    const MatrixBase< MatrixXd >&);

template DLLEXPORT DHomogTrans<Matrix<double, HOMOGENEOUS_TRANSFORM_SIZE, Dynamic>>::type dHomogTransInv(
    const Isometry3d&,
    const MatrixBase< Matrix<double, HOMOGENEOUS_TRANSFORM_SIZE, Dynamic> >&);

template DLLEXPORT Gradient< Matrix<double, TWIST_SIZE, Dynamic>, Dynamic, 1>::type dTransformSpatialMotion(
    const Isometry3d&,
    const MatrixBase< Matrix<double, TWIST_SIZE, Dynamic> >&,
    const MatrixBase< Matrix<double, HOMOGENEOUS_TRANSFORM_SIZE, Dynamic> >&,
    const MatrixBase<MatrixXd>&);

template DLLEXPORT Gradient< Matrix<double, TWIST_SIZE, 1>, Dynamic, 1>::type dTransformSpatialMotion(
    const Isometry3d&,
    const MatrixBase< Matrix<double, TWIST_SIZE, 1> >&,
    const MatrixBase< Matrix<double, HOMOGENEOUS_TRANSFORM_SIZE, Dynamic> >&,
    const MatrixBase< Matrix<double, TWIST_SIZE, Eigen::Dynamic> >&);

template DLLEXPORT TransformSpatial<Eigen::Matrix<double, TWIST_SIZE, 1>>::type crossSpatialMotion(
  const Eigen::MatrixBase<Eigen::Matrix<double, TWIST_SIZE, 1> >& a,
  const Eigen::MatrixBase<Eigen::Matrix<double, TWIST_SIZE, 1> >& b);

template DLLEXPORT TransformSpatial<Eigen::Matrix<double, TWIST_SIZE, TWIST_SIZE>>::type crossSpatialMotion(
  const Eigen::MatrixBase<Eigen::Matrix<double, TWIST_SIZE, 1> >& a,
  const Eigen::MatrixBase<Eigen::Matrix<double, TWIST_SIZE, TWIST_SIZE> >& b);

template DLLEXPORT TransformSpatial<Eigen::Matrix<double, TWIST_SIZE, Eigen::Dynamic>>::type crossSpatialMotion(
  const Eigen::MatrixBase<Eigen::Matrix<double, TWIST_SIZE, 1> >& a,
  const Eigen::MatrixBase<Eigen::Matrix<double, TWIST_SIZE, Eigen::Dynamic> >& b);

template DLLEXPORT TransformSpatial<Eigen::Matrix<double, TWIST_SIZE, 1>>::type crossSpatialForce(
  const Eigen::MatrixBase<Eigen::Matrix<double, TWIST_SIZE, 1> >& a,
  const Eigen::MatrixBase<Eigen::Matrix<double, TWIST_SIZE, 1> >& b);

template DLLEXPORT TransformSpatial<Eigen::Matrix<double, TWIST_SIZE, TWIST_SIZE>>::type crossSpatialForce(
  const Eigen::MatrixBase<Eigen::Matrix<double, TWIST_SIZE, 1> >& a,
  const Eigen::MatrixBase<Eigen::Matrix<double, TWIST_SIZE, TWIST_SIZE> >& b);

template DLLEXPORT Gradient< Matrix<double, TWIST_SIZE, Dynamic>, Dynamic, 1>::type dTransformSpatialForce(
    const Isometry3d&,
    const MatrixBase< Matrix<double, TWIST_SIZE, Dynamic> >&,
    const MatrixBase< Matrix<double, HOMOGENEOUS_TRANSFORM_SIZE, Dynamic> >&,
    const MatrixBase<MatrixXd>&);

template DLLEXPORT Gradient< Matrix<double, TWIST_SIZE, Dynamic>, Dynamic, 1>::type dTransformSpatialForce(
    const Isometry3d&,
    const MatrixBase< Matrix<double, TWIST_SIZE, Dynamic> >&,
    const MatrixBase<MatrixXd>&,
    const MatrixBase<MatrixXd>&);

template DLLEXPORT Gradient<Eigen::Matrix<double, 6, 1, 0, 6, 1>, Eigen::Block<Eigen::Matrix<double, 6, -1, 0, 6, -1>, 6, -1, true>::ColsAtCompileTime>::type dTransformSpatialForce(
    const Isometry3d& T,
    const Eigen::MatrixBase<Eigen::Matrix<double, 6, 1, 0, 6, 1> >& X,
    const Eigen::MatrixBase<Eigen::Matrix<double, 16, -1, 0, 16, -1> >& dT,
    const Eigen::MatrixBase<Eigen::Block<Eigen::Matrix<double, 6, -1, 0, 6, -1>, 6, -1, true> >& dX);

template DLLEXPORT Gradient<Eigen::Matrix<double, 6, 1, 0, 6, 1>, Eigen::Matrix<double, 6, -1, 0, 6, -1>::ColsAtCompileTime>::type dTransformSpatialForce(
    const Isometry3d& T,
    const Eigen::MatrixBase<Eigen::Matrix<double, 6, 1, 0, 6, 1>>& X,
    const Eigen::MatrixBase<Eigen::Matrix<double, 16, -1, 0, 16, -1>>& dT,
    const Eigen::MatrixBase<Eigen::Matrix<double, 6, -1, 0, 6, -1>>& dX);

template DLLEXPORT  void cylindrical2cartesian(const Matrix<double,3,1> &cylinder_axis, const Matrix<double,3,1> &cylinder_x_dir, const Matrix<double,3,1> & cylinder_origin, const Matrix<double,6,1> &x_cylinder, const Matrix<double,6,1> &v_cylinder, Matrix<double,6,1> &x_cartesian, Matrix<double,6,1> &v_cartesian, Matrix<double,6,6> &J, Matrix<double,6,1> &Jdotv );

template DLLEXPORT  void cartesian2cylindrical(const Matrix<double,3,1> &cylinder_axis, const Matrix<double,3,1> &cylinder_x_dir, const Matrix<double,3,1> & cylinder_origin, const Matrix<double,6,1> &x_cartesian, const Matrix<double,6,1> &v_cartesian, Matrix<double,6,1> &x_cylinder, Matrix<double,6,1> &v_cylinder, Matrix<double,6,6> &J, Matrix<double,6,1> &Jdotv );

template DLLEXPORT void angularvel2quatdotMatrix(const Eigen::MatrixBase<Vector4d>& q,
    Eigen::MatrixBase< Matrix<double, QUAT_SIZE, SPACE_DIMENSION> >& M,
    Eigen::MatrixBase< Gradient<Matrix<double, QUAT_SIZE, SPACE_DIMENSION>, QUAT_SIZE, 1>::type>* dM);
template DLLEXPORT void angularvel2quatdotMatrix(const Eigen::MatrixBase<Map<Vector4d>>& q,
    Eigen::MatrixBase< Matrix<double, QUAT_SIZE, SPACE_DIMENSION> >& M,
    Eigen::MatrixBase< Gradient<Matrix<double, QUAT_SIZE, SPACE_DIMENSION>, QUAT_SIZE, 1>::type>* dM);
template DLLEXPORT void angularvel2quatdotMatrix(const Eigen::MatrixBase< Eigen::Block<Eigen::Ref<Eigen::Matrix<double, -1, 1, 0, -1, 1> const, 0, Eigen::InnerStride<1> > const, 4, 1, false> >& q,
    Eigen::MatrixBase< Matrix<double, QUAT_SIZE, SPACE_DIMENSION> >& M,
    Eigen::MatrixBase< Gradient<Matrix<double, QUAT_SIZE, SPACE_DIMENSION>, QUAT_SIZE, 1>::type>* dM);

template DLLEXPORT void angularvel2rpydotMatrix(const Eigen::MatrixBase<Vector3d>& rpy,
    Eigen::MatrixBase< Matrix<double, RPY_SIZE, SPACE_DIMENSION> >& phi,
    Eigen::MatrixBase< Gradient<Matrix<double, RPY_SIZE, SPACE_DIMENSION>, RPY_SIZE, 1>::type>* dphi,
    Eigen::MatrixBase< Gradient<Matrix<double, RPY_SIZE, SPACE_DIMENSION>, RPY_SIZE, 2>::type>* ddphi);

template DLLEXPORT void angularvel2rpydotMatrix(const Eigen::MatrixBase<Vector3d>& rpy,
    Eigen::MatrixBase< Matrix<double, RPY_SIZE, SPACE_DIMENSION> >& phi,
    Eigen::MatrixBase< Matrix<double, Eigen::Dynamic, Eigen::Dynamic> >* dphi,
    Eigen::MatrixBase< Matrix<double, Eigen::Dynamic, Eigen::Dynamic> >* ddphi);

template DLLEXPORT void rpydot2angularvelMatrix(const Eigen::MatrixBase<Vector3d>& rpy,
    Eigen::MatrixBase<Eigen::Matrix<double, SPACE_DIMENSION, RPY_SIZE> >& E,
    Gradient<Matrix<double,SPACE_DIMENSION,RPY_SIZE>,RPY_SIZE,1>::type* dE);
template DLLEXPORT void rpydot2angularvelMatrix(const Eigen::MatrixBase<Map<Vector3d>>& rpy,
    Eigen::MatrixBase<Eigen::Matrix<double, SPACE_DIMENSION, RPY_SIZE> >& E,
    Gradient<Matrix<double,SPACE_DIMENSION,RPY_SIZE>,RPY_SIZE,1>::type* dE);
template DLLEXPORT void rpydot2angularvelMatrix(const Eigen::MatrixBase< Eigen::Block<Eigen::Ref<Eigen::Matrix<double, -1, 1, 0, -1, 1> const, 0, Eigen::InnerStride<1> > const, 3, 1, false> >& rpy,
    Eigen::MatrixBase<Eigen::Matrix<double, SPACE_DIMENSION, RPY_SIZE> >& E,
    Gradient<Matrix<double,SPACE_DIMENSION,RPY_SIZE>,RPY_SIZE,1>::type* dE);

template DLLEXPORT GradientVar<double, Eigen::Dynamic, SPACE_DIMENSION> angularvel2RepresentationDotMatrix(
    int rotation_type, const Eigen::MatrixBase<VectorXd>& qrot, int gradient_order);
template DLLEXPORT GradientVar<double, Eigen::Dynamic, SPACE_DIMENSION> angularvel2RepresentationDotMatrix(
    int rotation_type, const Eigen::MatrixBase< Eigen::Block<Eigen::Matrix<double, -1, -1, 0, -1, -1>, -1, 1, false> >& qrot, int gradient_order);
template DLLEXPORT GradientVar<double, -1, 3> angularvel2RepresentationDotMatrix<Eigen::Block<Eigen::Matrix<double, -1, 1, 0, -1, 1>, -1, 1, false> >(int, Eigen::MatrixBase<Eigen::Block<Eigen::Matrix<double, -1, 1, 0, -1, 1>, -1, 1, false> > const&, int);

template DLLEXPORT void quatdot2angularvelMatrix(const Eigen::MatrixBase<Vector4d>& q,
    Eigen::MatrixBase< Matrix<double, SPACE_DIMENSION, QUAT_SIZE> >& M,
    Gradient<Matrix<double, SPACE_DIMENSION, QUAT_SIZE>, QUAT_SIZE, 1>::type* dM);
template DLLEXPORT void quatdot2angularvelMatrix(const Eigen::MatrixBase<Map<Vector4d>>& q,
    Eigen::MatrixBase< Matrix<double, SPACE_DIMENSION, QUAT_SIZE> >& M,
    Gradient<Matrix<double, SPACE_DIMENSION, QUAT_SIZE>, QUAT_SIZE, 1>::type* dM);
template DLLEXPORT void quatdot2angularvelMatrix(const Eigen::MatrixBase< Eigen::Block<Eigen::Ref<Eigen::Matrix<double, -1, 1, 0, -1, 1> const, 0, Eigen::InnerStride<1> > const, 4, 1, false> >& q,
    Eigen::MatrixBase< Matrix<double, SPACE_DIMENSION, QUAT_SIZE> >& M,
    Gradient<Matrix<double, SPACE_DIMENSION, QUAT_SIZE>, QUAT_SIZE, 1>::type* dM);

template DLLEXPORT GradientVar<double,3,1> quat2expmap(const MatrixBase<Vector4d> &q, int gradient_order);
template DLLEXPORT GradientVar<double,3,1> quat2expmap(const MatrixBase<Map<Vector4d>> &q, int gradient_order);
