#pragma once

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include "drake/common/eigen_types.h"

namespace drake {
namespace math {
template <typename DerivedQ, typename DerivedB>
struct DecomposePositiveQuadraticFormType {
  static constexpr int kQsize =
      EigenSizeMinPreferFixed<DerivedQ::RowsAtCompileTime,
                              DerivedQ::ColsAtCompileTime>::value;
  static constexpr int kNumVar =
      EigenSizeMinPreferFixed<kQsize, DerivedB::RowsAtCompileTime>::value;
  typedef typename DerivedQ::Scalar ScalarType;
  typedef typename Eigen::Matrix<ScalarType, Eigen::Dynamic, kNumVar,
                                 Eigen::ColMajor, kNumVar, kNumVar>
      RType;
  typedef typename Eigen::Matrix<ScalarType, Eigen::Dynamic, 1, Eigen::ColMajor,
                                 kNumVar, 1>
      DType;
};

/**
 * Rewrite a quadratic form xᵀQx + bᵀx + c to
 * (Rx+d)ᵀ(Rx+d) + e.
 * where
 * RᵀR = Q
 * Rᵀd = b / 2
 * e = c - dᵀd
 * @tparam DerivedQ The type of Q
 * @tparam DerivedB The type of b
 */
template <typename DerivedQ, typename DerivedB>
std::tuple<
    typename DecomposePositiveQuadraticFormType<DerivedQ, DerivedB>::RType,
    typename DecomposePositiveQuadraticFormType<DerivedQ, DerivedB>::DType,
    double>
DecomposePositiveQuadraticForm(const DerivedQ& Q, const DerivedB& b, double c) {
  if (Q.rows() != Q.cols()) {
    throw std::runtime_error("Q should be a square matrix.");
  }
  if (b.rows() != Q.rows() || b.cols() != 1) {
    throw std::runtime_error("b is not in the right size.");
  }
  // The quadratic form xᵀQx + bᵀx + c can also be written as
  // [x]ᵀ * [Q   b/2] * [x]
  // [1]    [b/2   c]   [1]
  // We will call the matrix in the middle as P
  constexpr int PSize =
      DecomposePositiveQuadraticFormType<DerivedQ, DerivedB>::kNumVar ==
              Eigen::Dynamic
          ? Eigen::Dynamic
          : DecomposePositiveQuadraticFormType<DerivedQ, DerivedB>::kNumVar + 1;
  typedef Eigen::Matrix<typename DecomposePositiveQuadraticFormType<
                            DerivedQ, DerivedB>::ScalarType,
                        PSize, PSize>
      PType;
  PType P(Q.rows() + 1, Q.rows() + 1);
  P.block(0, 0, Q.rows(), Q.cols()) = Q;
  P.block(0, Q.cols(), Q.rows(), 1) = b / 2;
  P.block(Q.rows(), 0, 1, Q.cols()) = b.transpose() / 2;
  P(Q.rows(), Q.cols()) = c;

  typename DecomposePositiveQuadraticFormType<DerivedQ, DerivedB>::RType R;
  typename DecomposePositiveQuadraticFormType<DerivedQ, DerivedB>::DType d;
  double e{0};
  // P should be a positive semidefinite matrix, to make sure that the quadratic
  // form xᵀQx + bᵀx + c is always non-negative.
  // First do a Cholesky decomposition of P, as P = Uᵀ*U. If Cholesky fails,
  // then
  // P is not positive definite.
  Eigen::LLT<PType> llt(P);
  if (llt.info() == Eigen::Success) {
    // U is an upper diagonal matrix.
    // U = [U1 u2]
    //     [0  u3]
    R = llt.matrixU().nestedExpression().block(0, 0, Q.rows(), Q.cols());
    d = llt.matrixU().nestedExpression().block(0, Q.cols(), Q.rows(), 1);
    e = std::pow(llt.matrixU()(Q.rows(), Q.cols()), 2);
    return std::make_tuple(R, d, e);
  }
  // P should be positive semidefinite, so LDLT should be successful.
  Eigen::LDLT<PType> ldlt(P);
  if (ldlt.info() == Eigen::Success && ldlt.isPositive()) {
    return std::make_tuple(R, d, e);
  }
  throw std::runtime_error("The quadratic form is not positive semidefinite.");
};
}  // namespace math
}  // namespace drake
