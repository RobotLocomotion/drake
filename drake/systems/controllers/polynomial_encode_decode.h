#pragma once

#include <Eigen/Core>

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/lcmt_piecewise_polynomial.hpp"
#include "drake/lcmt_polynomial.hpp"
#include "drake/lcmt_polynomial_matrix.hpp"

void encodePolynomial(const Polynomial<double>& polynomial,
                      // NOLINTNEXTLINE(runtime/references)
                      drake::lcmt_polynomial& msg);

Polynomial<double> decodePolynomial(
    const drake::lcmt_polynomial& msg);

template <int RowsAtCompileTime, int ColsAtCompileTime>
void encodePolynomialMatrix(
    const Eigen::Matrix<Polynomial<double>, RowsAtCompileTime,
                        ColsAtCompileTime>& polynomial_matrix,
    // NOLINTNEXTLINE(runtime/references)
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

void encodePiecewisePolynomial(
    const PiecewisePolynomial<double>& piecewise_polynomial,
    // NOLINTNEXTLINE(runtime/references)
    drake::lcmt_piecewise_polynomial& msg);

PiecewisePolynomial<double> decodePiecewisePolynomial(
    const drake::lcmt_piecewise_polynomial& msg);
