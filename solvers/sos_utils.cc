#include "drake/solvers/sos_utils.h"

#include <iostream>

#include <Eigen/Core>

#include "drake/solvers/integer_lattice.h"

namespace {

ExponentList VerticalStack(const ExponentList & A, const ExponentList & B) {
  if (A.rows() == 0) {
    return B;
  }
  if (B.rows() == 0) {
    return A;
  }

  ExponentList Y(A.rows() + B.rows(), B.cols());
  Y << A, B;
  return Y;
}

ExponentList PairwiseSums(const ExponentList & points) {
  int n = points.rows();
  ExponentList sums((n*n + n)/2, points.cols());
  int cnt = 0;

  for (int i = 0; i < n; i++) {
    for (int j = i+1; j < n; j++) {
      sums.row(cnt++) = points.row(i) + points.row(j);
    }
  }
  return sums;
}

bool ContainsExponent(const ExponentList & A, const Exponent & B) {
  for (int i = 0; i < A.rows(); i++) {
    if (A.row(i) == B) {
      return true;
    }
  }
  return false;
}

ExponentList RemoveDiagonallyInconsistentExponents(
              const ExponentList & polynomial_support,
              ExponentList basis) {
  while (1) {
    ExponentList basis_next(basis.rows(), basis.cols()); int cnt = 0;
    ExponentList valid_squares = VerticalStack(PairwiseSums(basis),
                                               polynomial_support);
    for (int i = 0; i < basis.rows(); i++) {
      if (ContainsExponent(valid_squares, basis.row(i)*2)) {
        basis_next.row(cnt++) = basis.row(i);
      }
    }

    if (cnt == basis.rows()) {
      return basis;
    } else {
      basis = basis_next.block(0, 0, cnt, basis_next.cols());
    }
  }
}

struct Hyperplanes {
  Eigen::MatrixXi normal_vectors;
  Eigen::VectorXi max_dot_product;
  Eigen::VectorXi min_dot_product;
};

Hyperplanes RandomSupportingHyperplanes(ExponentList points,
             int num_hyperplanes) {
  Hyperplanes H;

  H.normal_vectors = Eigen::MatrixXi::Random(num_hyperplanes,
                                       points.cols())/100000;
  Eigen::MatrixXi dot_products = H.normal_vectors * points.transpose();

  H.max_dot_product = dot_products.rowwise().maxCoeff() / 2;
  H.min_dot_product = dot_products.rowwise().minCoeff() / 2;
  return H;
}

}  // namespace

ExponentList ConstructMonomialBasis(const ExponentList & M) {
  Eigen::VectorXi lower_bounds = M.colwise().minCoeff()/2;
  Eigen::VectorXi upper_bounds = M.colwise().maxCoeff()/2;

  Hyperplanes hyperplanes = RandomSupportingHyperplanes(M, 10);

  ExponentList points = EnumerateIntegerSolutions(
      hyperplanes.normal_vectors, hyperplanes.max_dot_product,
      lower_bounds, upper_bounds);

  return RemoveDiagonallyInconsistentExponents(M, points);
}


