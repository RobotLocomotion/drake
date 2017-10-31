#include "drake/util/lcmUtil.h"

#include "drake/math/quaternion.h"
#include "drake/math/rotation_matrix.h"
#include "drake/util/drakeUtil.h"

using Eigen::Dynamic;
using Eigen::Isometry3d;
using Eigen::Map;
using Eigen::VectorXd;

void EncodeVector3d(const Eigen::Ref<const Eigen::Vector3d>& vec,
                    bot_core::vector_3d_t& msg) {
  msg.x = vec[0];
  msg.y = vec[1];
  msg.z = vec[2];
}

Eigen::Vector3d DecodeVector3d(const bot_core::vector_3d_t& msg) {
  return Eigen::Vector3d(msg.x, msg.y, msg.z);
}

void EncodeQuaternion(const Eigen::Ref<const Eigen::Vector4d>& vec,
                      bot_core::quaternion_t& msg) {
  msg.w = vec[0];
  msg.x = vec[1];
  msg.y = vec[2];
  msg.z = vec[3];
}

Eigen::Vector4d DecodeQuaternion(const bot_core::quaternion_t& msg) {
  return Eigen::Vector4d(msg.w, msg.x, msg.y, msg.z);
}

void EncodePose(const Eigen::Isometry3d& pose, bot_core::position_3d_t& msg) {
  auto rotation = drake::math::rotmat2quat(pose.linear());
  EncodeQuaternion(rotation, msg.rotation);
  EncodeVector3d(pose.translation(), msg.translation);
}

Eigen::Isometry3d DecodePose(const bot_core::position_3d_t& msg) {
  Isometry3d ret;
  ret.translation() = DecodeVector3d(msg.translation);
  auto quaternion = DecodeQuaternion(msg.rotation);
  ret.linear() = drake::math::quat2rotmat(quaternion);
  ret.makeAffine();
  return ret;
}

void EncodeTwist(const Eigen::Ref<const drake::TwistVector<double>>& twist,
                 bot_core::twist_t& msg) {
  EncodeVector3d(twist.head<3>(), msg.angular_velocity);
  EncodeVector3d(twist.tail<3>(), msg.linear_velocity);
}

drake::TwistVector<double> DecodeTwist(const bot_core::twist_t& msg) {
  drake::TwistVector<double> ret;
  ret.head<3>() = DecodeVector3d(msg.angular_velocity);
  ret.tail<3>() = DecodeVector3d(msg.linear_velocity);
  return ret;
}

void encodePolynomial(const Polynomial<double>& polynomial,
                      drake::lcmt_polynomial& msg) {
  eigenVectorToStdVector(polynomial.GetCoefficients(), msg.coefficients);
  msg.num_coefficients = polynomial.GetNumberOfCoefficients();
}

Polynomial<double> decodePolynomial(const drake::lcmt_polynomial& msg) {
  Map<const VectorXd> coefficients(msg.coefficients.data(),
                                   msg.coefficients.size());
  return Polynomial<double>(coefficients);
}

void encodePiecewisePolynomial(
    const PiecewisePolynomial<double>& piecewise_polynomial,
    drake::lcmt_piecewise_polynomial& msg) {
  msg.num_segments = piecewise_polynomial.getNumberOfSegments();
  msg.num_breaks = piecewise_polynomial.getNumberOfSegments() + 1;
  msg.breaks = piecewise_polynomial.getSegmentTimes();
  msg.polynomial_matrices.resize(piecewise_polynomial.getNumberOfSegments());
  for (int i = 0; i < piecewise_polynomial.getNumberOfSegments(); ++i) {
    encodePolynomialMatrix<Eigen::Dynamic, Eigen::Dynamic>(
        piecewise_polynomial.getPolynomialMatrix(i),
        msg.polynomial_matrices[i]);
  }
}

PiecewisePolynomial<double> decodePiecewisePolynomial(
    const drake::lcmt_piecewise_polynomial& msg) {
  typedef PiecewisePolynomial<double>::PolynomialMatrix PolynomialMatrix;
  std::vector<PolynomialMatrix> polynomial_matrices;
  for (size_t i = 0; i < msg.polynomial_matrices.size(); ++i) {
    polynomial_matrices.push_back(
        decodePolynomialMatrix<Dynamic, Dynamic>(msg.polynomial_matrices[i]));
  }
  return PiecewisePolynomial<double>(polynomial_matrices, msg.breaks);
}
