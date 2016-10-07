#pragma once

#include <Eigen/Core>
#include <iostream>
#include "drake/systems/trajectories/PiecewisePolynomial.h"

#include "drake/common/drake_export.h"
#include "bot_core/position_3d_t.hpp"
#include "bot_core/quaternion_t.hpp"
#include "bot_core/twist_t.hpp"
#include "bot_core/vector_3d_t.hpp"
#include "drake/lcmt_piecewise_polynomial.hpp"
#include "drake/lcmt_polynomial.hpp"
#include "drake/lcmt_polynomial_matrix.hpp"
#include "drake/lcmt_qp_controller_input.hpp"

DRAKE_EXPORT void EncodeVector3d(
    const Eigen::Ref<const Eigen::Vector3d>& vec, bot_core::vector_3d_t& msg);

DRAKE_EXPORT Eigen::Vector3d DecodeVector3d(
    const bot_core::vector_3d_t& msg);

DRAKE_EXPORT void EncodeQuaternion(
    const Eigen::Ref<const Eigen::Vector4d>& vec, bot_core::quaternion_t& msg);

DRAKE_EXPORT Eigen::Vector4d DecodeQuaternion(
    const bot_core::quaternion_t& msg);

// Note that bot_core::position_3d_t is badly named.
DRAKE_EXPORT void EncodePose(const Eigen::Isometry3d& pose,
                                    bot_core::position_3d_t& msg);

// Note that bot_core::position_3d_t is badly named.
DRAKE_EXPORT Eigen::Isometry3d DecodePose(
    const bot_core::position_3d_t& msg);

DRAKE_EXPORT void EncodeTwist(
    const Eigen::Ref<const drake::TwistVector<double>>& twist,
    bot_core::twist_t& msg);

DRAKE_EXPORT drake::TwistVector<double> DecodeTwist(
    const bot_core::twist_t& msg);

DRAKE_EXPORT void encodePolynomial(const Polynomial<double>& polynomial,
                                          drake::lcmt_polynomial& msg);

DRAKE_EXPORT Polynomial<double> decodePolynomial(
    const drake::lcmt_polynomial& msg);

template <int RowsAtCompileTime, int ColsAtCompileTime>
void encodePolynomialMatrix(
    const Eigen::Matrix<Polynomial<double>, RowsAtCompileTime,
                        ColsAtCompileTime>& polynomial_matrix,
    drake::lcmt_polynomial_matrix& msg) {
  msg.polynomials.clear();
  msg.polynomials.resize(polynomial_matrix.rows());
  for (int row = 0; row < polynomial_matrix.rows(); ++row) {
    auto& polynomial_msg_row = msg.polynomials[row];
    polynomial_msg_row.resize(polynomial_matrix.cols());
    for (int col = 0; col < polynomial_matrix.cols(); ++col) {
      encodePolynomial(polynomial_matrix(row, col), polynomial_msg_row[col]);
    }
  }
  msg.rows = polynomial_matrix.rows();
  msg.cols = polynomial_matrix.cols();
}

template <int RowsAtCompileTime, int ColsAtCompileTime>
Eigen::Matrix<Polynomial<double>, RowsAtCompileTime, ColsAtCompileTime>
decodePolynomialMatrix(const drake::lcmt_polynomial_matrix& msg) {
  Eigen::Matrix<Polynomial<double>, RowsAtCompileTime, ColsAtCompileTime> ret(
      msg.rows, msg.cols);
  for (int row = 0; row < msg.rows; ++row) {
    for (int col = 0; col < msg.cols; ++col) {
      ret(row, col) = decodePolynomial(msg.polynomials[row][col]);
    }
  }
  return ret;
}

DRAKE_EXPORT void encodePiecewisePolynomial(
    const PiecewisePolynomial<double>& piecewise_polynomial,
    drake::lcmt_piecewise_polynomial& msg);

DRAKE_EXPORT PiecewisePolynomial<double> decodePiecewisePolynomial(
    const drake::lcmt_piecewise_polynomial& msg);

DRAKE_EXPORT void verifySubtypeSizes(
    drake::lcmt_support_data& support_data);
DRAKE_EXPORT void verifySubtypeSizes(
    drake::lcmt_qp_controller_input& qp_input);
