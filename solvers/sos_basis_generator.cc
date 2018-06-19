#include "drake/solvers/sos_basis_generator.h"

#include <vector>

#include <Eigen/Core>

#include "drake/common/symbolic.h"
#include "drake/solvers/integer_inequality_solver.h"

namespace drake {
namespace solvers {
namespace {
// Anonymous namespace containing collection of utility functions
using Variable = drake::symbolic::Variable;
using Monomial = drake::symbolic::Monomial;
using Variables = drake::symbolic::Variables;
using MonomialVector = drake::VectorX<symbolic::Monomial>;
using Exponent = Eigen::RowVectorXi;
using ExponentList = Eigen::MatrixXi;

// Given a list of exponents and variables, returns a vector of monomials.
// Ex: if exponents = [0, 1;1, 2], and vars = [x(0), x(1)], then the vector
// [x(1); x(0)* x(1)²] is returned.
MonomialVector ExponentsToMonomials(const ExponentList& exponents,
                                    const drake::VectorX<Variable>& vars) {
  MonomialVector monomials(exponents.rows());
  for (int i = 0; i < exponents.rows(); i++) {
    monomials(i) = Monomial(vars, exponents.row(i));
  }
  return monomials;
}

// Returns a list of all exponents that appear in a polynomial p.
// E.g., given p = 1 + 2x₀² + 3x₀*x₁², returns [0, 0; 2, 0; 1, 2];
ExponentList GetPolynomialExponents(const drake::symbolic::Polynomial& p) {
  ExponentList exponents(p.monomial_to_coefficient_map().size(),
                         p.indeterminates().size());
  int row = 0;
  for (const auto& m : p.monomial_to_coefficient_map()) {
    int col = 0;
    for (const auto& var : p.indeterminates()) {
      exponents(row, col++) = m.first.degree(var);
    }
    row++;
  }
  return exponents;
}

ExponentList VerticalStack(const ExponentList& A, const ExponentList& B) {
  DRAKE_ASSERT(A.cols() == B.cols());
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

ExponentList PairwiseSums(const ExponentList& exponents) {
  int n = exponents.rows();
  ExponentList sums((n * n + n) / 2, exponents.cols());
  int cnt = 0;
  for (int i = 0; i < n; i++) {
    // Note: counter starts at i+1 to omit a+b when a = b.
    for (int j = i + 1; j < n; j++) {
      sums.row(cnt++) = exponents.row(i) + exponents.row(j);
    }
  }
  return sums;
}

bool ContainsExponent(const ExponentList& A, const Exponent& B) {
  DRAKE_DEMAND(B.rows() == 1 && B.cols() == A.cols());
  for (int i = 0; i < A.rows(); i++) {
    if (A.row(i) == B) {
      return true;
    }
  }
  return false;
}

/* Removes exponents of the monomials that aren't diagonally-consistent with
 * respect to the polynomial p and the given monomial basis.  A monomial is
 * diagonally-consistent if its square appears in p, or its square equals a
 * product of monomials in the basis; see, e.g., "Pre- and Post-Processing
 * Sum-of-Squares Programs in Practice Johan Löfberg, IEEE Transactions on
 * Automatic Control, 2009."
*/
void RemoveDiagonallyInconsistentExponents(const ExponentList& exponents_of_p,
                                           ExponentList* exponents_of_basis) {
  while (1) {
    ExponentList exponents_to_keep(exponents_of_basis->rows(),
                                   exponents_of_basis->cols());
    int cnt = 0;
    ExponentList valid_squares =
        VerticalStack(PairwiseSums(*exponents_of_basis), exponents_of_p);
    for (int i = 0; i < exponents_of_basis->rows(); i++) {
      if (ContainsExponent(valid_squares, exponents_of_basis->row(i) * 2)) {
        exponents_to_keep.row(cnt++) = exponents_of_basis->row(i);
      }
    }

    if (cnt == exponents_of_basis->rows()) {
      return;
    } else {
      exponents_of_basis->resize(cnt, exponents_of_basis->cols());
      exponents_of_basis->block(0, 0, cnt, exponents_of_basis->cols()) =
          exponents_to_keep.block(0, 0, cnt, exponents_to_keep.cols());
    }
  }
}

struct Hyperplanes {
  Eigen::MatrixXi normal_vectors;  // Each row contains a normal vector.
  Eigen::VectorXi max_dot_product;
  Eigen::VectorXi min_dot_product;
};

/* Finding random supporting hyperplanes of 1/2 P, where P is the Newton
 * polytope of the polynomial p (i.e., the convex hull of its exponents).
*/
Hyperplanes RandomSupportingHyperplanes(const ExponentList& exponents_of_p,
                                        int num_hyperplanes) {
  Hyperplanes H;

  int scale = RAND_MAX / 100;
  H.normal_vectors =
      Eigen::MatrixXi::Random(num_hyperplanes, exponents_of_p.cols()) / scale;
  Eigen::MatrixXi dot_products = H.normal_vectors * exponents_of_p.transpose();
  H.max_dot_product = dot_products.rowwise().maxCoeff() / 2;
  H.min_dot_product = dot_products.rowwise().minCoeff() / 2;

  return H;
}
}  // namespace

/* Finds monomial basis for given polynomial p. We provide two implementions:
 * the first takes as input the polynomial p, and the second takes as input the
 * exponents of p.*/
ExponentList ConstructMonomialBasis(const ExponentList& exponents_of_p) {
  Eigen::VectorXi lower_bounds = exponents_of_p.colwise().minCoeff() / 2;
  Eigen::VectorXi upper_bounds = exponents_of_p.colwise().maxCoeff() / 2;

  int num_hyperplanes = 10;
  // Fix the random seed so code is deterministic.
  unsigned int random_seed = 1;
  std::srand(random_seed);
  Hyperplanes hyperplanes =
      RandomSupportingHyperplanes(exponents_of_p, num_hyperplanes);

  ExponentList basis_exponents = drake::solvers::EnumerateIntegerSolutions(
      hyperplanes.normal_vectors, hyperplanes.max_dot_product, lower_bounds,
      upper_bounds);

  RemoveDiagonallyInconsistentExponents(exponents_of_p, &basis_exponents);
  return basis_exponents;
}

MonomialVector ConstructMonomialBasis(const drake::symbolic::Polynomial& p) {
  drake::VectorX<Variable> vars(p.indeterminates().size());
  int cnt = 0;
  for (auto& var : p.indeterminates()) {
    vars(cnt++) = var;
  }
  return ExponentsToMonomials(ConstructMonomialBasis(GetPolynomialExponents(p)),
                              vars);
}
}  // namespace solvers
}  // namespace drake
