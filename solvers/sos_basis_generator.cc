#include "drake/solvers/sos_basis_generator.h"

#include <vector>

#include <Eigen/Core>

#include "drake/solvers/integer_inequality_solver.h"
namespace drake {
namespace solvers {
namespace {
// Anonymous namespace containing collection of utility functions
using Variable = symbolic::Variable;
using Monomial = symbolic::Monomial;
using Variables = symbolic::Variables;
using MonomialVector = VectorX<symbolic::Monomial>;
using Exponent = Eigen::RowVectorXi;
using ExponentList = Eigen::Matrix<int, -1, -1, Eigen::RowMajor>;

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
  const Variables indeterminates{p.indeterminates()};
  ExponentList exponents(p.monomial_to_coefficient_map().size(),
                         indeterminates.size());
  int row = 0;
  for (const auto& m : p.monomial_to_coefficient_map()) {
    int col = 0;
    for (const auto& var : indeterminates) {
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
  ExponentList sums((n * n - n) / 2, exponents.cols());
  int cnt = 0;
  for (int i = 0; i < n; i++) {
    // Note: counter starts at i+1 to omit a+b when a = b.
    for (int j = i + 1; j < n; j++) {
      sums.row(cnt++) = exponents.row(i) + exponents.row(j);
    }
  }
  return sums;
}

// Returns true if the first num_rows of A contains B.
bool ContainsExponent(const ExponentList& A, int num_rows, const Exponent& B) {
  DRAKE_ASSERT(B.rows() == 1 && B.cols() == A.cols());
  DRAKE_ASSERT((num_rows >= 0) && (num_rows <= A.rows()));
  for (int i = 0; i < num_rows; i++) {
    if (A.row(i) == B) {
      return true;
    }
  }
  return false;
}

/* Intersection(A, B) removes duplicate rows from B and any row that doesn't also
 * appear in A.  For example, given A = [1, 0; 0, 1; 1, 1] and B = [1, 0; 1, 1;
 * 1, 1;], it overwrites B with [1, 0; 1, 1]. */

void Intersection(const ExponentList& A, ExponentList* B) {
  DRAKE_ASSERT(A.cols() == B->cols());
  int index = 0;
  for (int i = 0; i < B->rows(); i++) {
    if ((ContainsExponent(A, A.rows(), B->row(i))) &&
        !(ContainsExponent(*B, index, B->row(i)))) {
      B->row(index++) = B->row(i);
    }
  }
  B->conservativeResize(index, Eigen::NoChange);
}

/* Removes exponents of the monomials that aren't diagonally-consistent with
 * respect to the polynomial p and the given monomial basis.  A monomial is
 * diagonally-consistent if its square appears in p, or its square equals a
 * product of monomials in the basis; see, e.g., "Pre- and Post-Processing
 * Sum-of-Squares Programs in Practice Johan Löfberg, IEEE Transactions on
 * Automatic Control, 2009." After execution, all exponents of inconsistent
 * monomials are removed from exponents_of_basis.
*/
void RemoveDiagonallyInconsistentExponents(const ExponentList& exponents_of_p,
                                           ExponentList* exponents_of_basis) {
  while (1) {
    int num_exponents = exponents_of_basis->rows();

    ExponentList valid_squares =
        VerticalStack(PairwiseSums(*exponents_of_basis), exponents_of_p);

    (*exponents_of_basis) = (*exponents_of_basis) * 2;
    Intersection(valid_squares, exponents_of_basis);
    (*exponents_of_basis) = (*exponents_of_basis) / 2;

    if (exponents_of_basis->rows() == num_exponents) {
      return;
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

Hyperplanes RandomSupportingHyperplanes(const ExponentList& exponents_of_p) {
  Hyperplanes H;


  // get_random() samples uniformly between normal_vector_component_min/max.
  const int normal_vector_component_min  = 1;
  const int normal_vector_component_max  = 20;
  std::default_random_engine generator;
  std::uniform_int_distribution<int> distribution(normal_vector_component_min,
                                                  normal_vector_component_max);
  auto get_random = [&]() { return distribution(generator); };


  // Number of hyperplanes currently picked heuristically.
  // TODO(frankpermenter): Pick using degree, number of variables,
  // and length of exponents_of_p.
  int num_hyperplanes = 10*exponents_of_p.cols();

  //  We generate nonnegative or nonpositive columns so that call to
  //  EnumerateIntegerSolutions is more efficient.
  H.normal_vectors = Eigen::MatrixXi(num_hyperplanes, exponents_of_p.cols());
  for (int i = 0; i < H.normal_vectors.cols(); i++) {
    const int sign = (i % 2) ? -1 : 1;
    H.normal_vectors.col(i) << sign*Eigen::VectorXi::NullaryExpr(
                                                  num_hyperplanes, get_random);
  }

  Eigen::MatrixXi dot_products = H.normal_vectors * exponents_of_p.transpose();
  H.max_dot_product = dot_products.rowwise().maxCoeff() / 2;
  H.min_dot_product = dot_products.rowwise().minCoeff() / 2;

  return H;
}

ExponentList ConstructMonomialBasis(const ExponentList& exponents_of_p) {
  Eigen::VectorXi lower_bounds = exponents_of_p.colwise().minCoeff() / 2;
  Eigen::VectorXi upper_bounds = exponents_of_p.colwise().maxCoeff() / 2;

  // Note: RandomSupportingHyperplanes is actually deterministic due to
  // its internal initialization of the random number seed.
  Hyperplanes hyperplanes = RandomSupportingHyperplanes(exponents_of_p);

  // We check the inequalities in two batches to allow for internal
  // infeasibility propagation inside of EnumerateIntegerSolutions,
  // which is done only if A has a column that is elementwise nonnegative
  // or nonpositive. (This condition never holds if we check the
  // inequalities in one batch, since A = [normal_vectors;-normal_vectors].)
  ExponentList basis_exponents_1 = drake::solvers::EnumerateIntegerSolutions(
      hyperplanes.normal_vectors, hyperplanes.max_dot_product, lower_bounds,
      upper_bounds);

  ExponentList basis_exponents = drake::solvers::EnumerateIntegerSolutions(
      -hyperplanes.normal_vectors, -hyperplanes.min_dot_product, lower_bounds,
      upper_bounds);

  Intersection(basis_exponents_1, &basis_exponents);
  RemoveDiagonallyInconsistentExponents(exponents_of_p, &basis_exponents);
  return basis_exponents;
}

}  // namespace


MonomialVector ConstructMonomialBasis(const drake::symbolic::Polynomial& p) {
  const Variables indeterminates{p.indeterminates()};
  drake::VectorX<Variable> vars(indeterminates.size());
  int cnt = 0;
  for (auto& var : indeterminates) {
    vars(cnt++) = var;
  }
  return ExponentsToMonomials(ConstructMonomialBasis(GetPolynomialExponents(p)),
                              vars);
}
}  // namespace solvers
}  // namespace drake
