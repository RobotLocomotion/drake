#ifndef UTIL_LCMUTIL_H_
#define UTIL_LCMUTIL_H_

#include <Eigen/Core>
#include <iostream>
#include "PiecewisePolynomial.h"
#include "lcmtypes/drake/lcmt_polynomial.hpp"
#include "lcmtypes/drake/lcmt_polynomial_matrix.hpp"
#include "lcmtypes/drake/lcmt_piecewise_polynomial.hpp"
#include "lcmtypes/drake/lcmt_qp_controller_input.hpp"

#undef DLLEXPORT
#if defined(WIN32) || defined(WIN64)
  #if defined(drakeLCMUtil_EXPORTS)
    #define DLLEXPORT __declspec( dllexport )
  #else
    #define DLLEXPORT __declspec( dllimport )
  #endif
#else
    #define DLLEXPORT
#endif

DLLEXPORT void encodePolynomial(const Polynomial<double>& polynomial, drake::lcmt_polynomial& msg);

DLLEXPORT Polynomial<double> decodePolynomial(const drake::lcmt_polynomial& msg);

template <int RowsAtCompileTime, int ColsAtCompileTime>
void encodePolynomialMatrix(const Eigen::Matrix<Polynomial<double>, RowsAtCompileTime, ColsAtCompileTime>& polynomial_matrix, drake::lcmt_polynomial_matrix& msg)
{
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
Eigen::Matrix<Polynomial<double>, RowsAtCompileTime, ColsAtCompileTime> decodePolynomialMatrix(const drake::lcmt_polynomial_matrix& msg)
{
  Eigen::Matrix<Polynomial<double>, RowsAtCompileTime, ColsAtCompileTime> ret(msg.rows, msg.cols);
  for (int row = 0; row < msg.rows; ++row) {
    for (int col = 0; col < msg.cols; ++col) {
      ret(row, col) = decodePolynomial(msg.polynomials[row][col]);
    }
  }
  return ret;
}

DLLEXPORT void encodePiecewisePolynomial(const PiecewisePolynomial<double>& piecewise_polynomial, drake::lcmt_piecewise_polynomial& msg);

DLLEXPORT PiecewisePolynomial<double> decodePiecewisePolynomial(const drake::lcmt_piecewise_polynomial& msg);

DLLEXPORT void verifySubtypeSizes(drake::lcmt_support_data &support_data);
DLLEXPORT void verifySubtypeSizes(drake::lcmt_qp_controller_input &qp_input);

#endif /* UTIL_LCMUTIL_H_ */
