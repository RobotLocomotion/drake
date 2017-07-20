#include "drake/solvers/sos_util.h"

#include <algorithm>
#include <cmath>
#include <exception>
#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <random>
#include <set>
#include <string>
#include <utility>

#include <Eigen/Core>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/common/hash.h"
#include "drake/common/symbolic.h"

namespace drake {
namespace solvers {

using std::map;
using std::set;
using Eigen::MatrixXi;
using Eigen::VectorXi;
using symbolic::Monomial;
using symbolic::Variable;
using symbolic::Variables;

VectorX<Monomial> ConstructGramBasis(const symbolic::Polynomial& p) {
  if (p.TotalDegree() % 2 == 1) {  // catches off polynomials
    // TODO(FischerGundlach) Catch all obvious non-SOS polynomial, i.e.
    // x_0^3*x_1^5 is clearly not SOS.
    throw std::runtime_error("Polynomial is odd and thus can't be SOS.");
  } else if (p.indeterminates().empty()) {  // catches constant polynomials
    // TODO(FischerGundlach) According to Eigen documentation, this should take
    // a reference, i,e. Matrix (const Scalar &x). Why does it take the rvalue
    // reference?
    return Vector1<Monomial>(Monomial());
  } else {
    std::pair<MatrixXi, map<Variable, int>> pair{
        build_gram_basis::PolynomialToSupport(p)};
    const MatrixXi pow{pair.first};
    MatrixXi mpow{build_gram_basis::GenerateExponentBoundPolytope(pow)};

    mpow = build_gram_basis::RandomlyPruneSupport(pow, mpow);
    mpow = build_gram_basis::CheckDiagonalConsistency(pow, mpow);

    return build_gram_basis::SupportToGramBasis(mpow, pair.second);
  }
}

namespace build_gram_basis {

MatrixXi GenerateCrossTerms(const MatrixXi& mpow) {
  MatrixXi crossterms(symbolic::NChooseK(mpow.rows(), 2), mpow.cols());
  int crossindex{0};
  for (int i = 0; i < mpow.rows() - 1; ++i) {
    for (int j = i + 1; j < mpow.rows(); ++j) {
      crossterms.row(crossindex) = mpow.row(i) + mpow.row(j);
      crossindex++;
    }
  }

  return UniqueRows(crossterms);
}

MatrixXi CheckDiagonalConsistency(const MatrixXi& pow, MatrixXi mpow) {
  // pow.cols() == mpow.cols(), otherwise they are not in the same amount of
  // indeterminates
  // mpow.rows() != 0 otherwise there are no monomials/ points in the support
  // mpow.cols() != 0 otherwise, the monomials are in 0 indeterminates, i.e.
  // the polynomial needs to be constant or of degree 1.
  // These cases should be caught in ConstructGramBasis already.
  DRAKE_ASSERT(pow.cols() == mpow.cols() && mpow.cols() != 0 &&
               mpow.rows() != 0);

  MatrixXi mpow_sqr{RowIntersect(2 * mpow, pow) / 2};

  auto N = mpow.rows();
  while (1) {
    MatrixXi crossterms = GenerateCrossTerms(mpow);

    if (crossterms.rows() != 0) {
      // TODO(FischerGndlach) The following rows can be squashed + do I need
      // temp?
      MatrixXi intersect_terms = RowIntersect(2 * mpow, crossterms);
      intersect_terms = intersect_terms / 2;
      MatrixXi temp(mpow_sqr.rows() + intersect_terms.rows(), mpow.cols());

      temp << mpow_sqr, intersect_terms;

      mpow = UniqueRows(temp);
    } else {
      mpow = mpow_sqr;
    }
    // If the last iteration did not change the size, then break.
    if (mpow.rows() == N) {
      break;
    } else {
      N = mpow.rows();
    }
  }

  return mpow;
}

MatrixXi GenerateExponentBoundPolytope(const MatrixXi& pow) {
  // TODO(FischerGundlach) Implement quicker combinatorial creation (dont rely
  // on the slow creation of monomialBasis).

  // pow.rows() != 0 otherwise there are no monomials/ points in the support
  // pow.cols() != 0 otherwise, the monomials are in 0 indeterminates, i.e.
  // the polynomial must NOT be constant.
  DRAKE_ASSERT(pow.cols() != 0 && pow.rows() != 0);

  Variables indeterminates;
  for (int i = 0; i < pow.cols(); ++i) {
    indeterminates.insert(Variable(std::to_string(i)));
  }
  const int maximum_degree_in_pow{
      (pow.rowwise().sum()).colwise().maxCoeff()[0]};
  const int maximum_degree_in_m_pow{
      static_cast<int>(std::floor(maximum_degree_in_pow / 2))};

  // maximum_degree_in_m_pow >= 1 otherwise pow is constant or clearly not SOS.
  DRAKE_ASSERT(maximum_degree_in_m_pow >= 1);

  const VectorX<Monomial> m{
      drake::symbolic::MonomialBasis(indeterminates, maximum_degree_in_m_pow)};
  symbolic::Polynomial::MapType p_map;
  for (int i = 0; i < m.size(); ++i) {
    p_map.emplace(m(i), 1);
  }

  return PolynomialToSupport(symbolic::Polynomial(p_map)).first;
}

std::pair<MatrixXi, map<Variable, int>> PolynomialToSupport(
    const symbolic::Polynomial& p) {
  Variables indeterminates{p.indeterminates()};
  map<Variable, int> variable_to_position_map;

  int column{0};
  for (Variables::iterator it = indeterminates.begin();
       it != indeterminates.end(); ++it) {
    variable_to_position_map.insert(std::pair<Variable, int>(*it, column));
    ++column;
  }

  set<Monomial, symbolic::GradedReverseLexOrder<std::less<Variable>>>
      monomials_in_poly;
  for (auto& m : p.monomial_to_coefficient_map()) {
    monomials_in_poly.insert(m.first);
  }

  MatrixXi support(monomials_in_poly.size(), indeterminates.size());
  support *= 0;
  int row{0};
  for (auto& mono : monomials_in_poly) {
    map<Variable, int> powers{mono.get_powers()};
    for (auto variable_2_power : powers) {
      support(row, variable_to_position_map[variable_2_power.first]) =
          variable_2_power.second;
    }
    ++row;
  }

  return std::make_pair(support, variable_to_position_map);
}

MatrixXi RandomlyPruneSupport(const MatrixXi& pow, MatrixXi mpow,
                              const int num_hyper_planes, const int seed) {
  // pow.cols() == mpow.cols(), otherwise they are not in the same amount of
  // indeterminates
  // mpow.rows() != 0 otherwise there are no monomials/ points in the support
  // mpow.cols() != 0 otherwise, the monomials are in 0 indeterminates, i.e.
  // the polynomial needs to be constant or of degree 1.
  // These cases should be caught in ConstructGramBasis already.
  DRAKE_ASSERT(pow.cols() == mpow.cols() && mpow.cols() != 0 &&
               mpow.rows() != 0);

  VectorXi w{mpow.cols()};
  VectorXi y{mpow.rows()};

  MatrixXi new_mpow(mpow.rows(), mpow.cols());

  double lower_bound, upper_bound;

  // Seed and range [0,99] are chosen arbitrarily.
  std::mt19937 mt(seed);
  std::uniform_int_distribution<int> dist(0, 99);
  for (int i = 0; i < num_hyper_planes; ++i) {
    for (int j = 0; j < w.rows(); ++j) {
      w(j) = dist(mt);
    }
    lower_bound = (pow * w).minCoeff();
    upper_bound = (pow * w).maxCoeff();

    y = 2 * (mpow * w);

    // TODO(FischerGundlach) Rewrite this block to use remove row! (But maybe
    // this implementation is quicker!)
    // reduce rows so that if an entry of y is between the bounds, then keep
    // that row. Otherwise, remove that row.
    int exterior_rows = 0;
    for (int k = 0; k < y.rows(); ++k) {
      if (y(k) >= lower_bound && y(k) <= upper_bound) {
        new_mpow.row(k - exterior_rows) = mpow.row(k);
      } else {
        exterior_rows++;
      }
    }
    new_mpow.conservativeResize(y.rows() - exterior_rows, new_mpow.cols());

    mpow = new_mpow;
    if (mpow.rows() < 100) {
      return mpow;
    }
  }
  return mpow;
}

VectorX<Monomial> SupportToGramBasis(
    const MatrixXi& mpow, const map<Variable, int>& variable_to_position_map) {
  VectorX<Monomial> gram_basis(mpow.rows());

  for (int i = 0; i < static_cast<int>(mpow.rows()); ++i) {
    map<Variable, int> powers;
    for (auto& m : variable_to_position_map) {
      powers.insert(std::make_pair(m.first, mpow(i, m.second)));
    }
    gram_basis(i) = Monomial(powers);
  }

  return gram_basis;
}

// From here on definition of helper functions.

MatrixXi RemoveRows(const MatrixXi& mat, const set<int>& rows_to_remove) {
  MatrixXi reduced_matrix(mat.rows() - rows_to_remove.size(), mat.cols());
  // now rows to remove is sorted, so populating reduced_matrix is easy
  int removed_rows{0};
  for (int i = 0; i < mat.rows(); i++) {
    if (rows_to_remove.find(i) == (rows_to_remove.end())) {
      reduced_matrix.row(i - removed_rows) = mat.row(i);
    } else {
      removed_rows++;
    }
  }

  return reduced_matrix;
}

MatrixXi RowIntersect(const MatrixXi& mat1, const MatrixXi& mat2) {
  if (mat1.cols() != mat2.cols()) {
    return MatrixXi();
  }

  MatrixXi ret_mat(mat1.rows(), mat2.cols());
  int num_match{0};
  for (int i = 0; i < mat1.rows(); ++i) {
    for (int j = 0; j < mat2.rows(); ++j) {
      if (mat1.row(i) == mat2.row(j)) {
        ret_mat.row(num_match) = mat1.row(i);
        num_match++;
      }
    }
  }

  ret_mat.conservativeResize(num_match, ret_mat.cols());
  return ret_mat;
}

MatrixXi UniqueRows(const MatrixXi& mat) {
  set<int> row_duplicates;
  bool no_row_duplicate{true};

  // TODO(FischerGundlach) Look for faster ways to cull the duplicate rows, i.e.
  // would it be better to remove the row immediately?
  for (int i = 0; i < mat.rows() - 1; i++) {
    for (int j = i + 1; j < mat.rows(); j++) {
      if (mat.row(i) == mat.row(j)) {
        row_duplicates.insert(j);
        no_row_duplicate = false;
      }
    }
  }

  if (no_row_duplicate == true) {
    return mat;
  } else {
    return RemoveRows(mat, row_duplicates);
  }
}

}  // namespace build_gram_basis
}  // namespace solvers
}  // namespace drake
