#include "drake/solvers/sos_util.h"

#include <algorithm>
#include <functional>
#include <iostream>
#include <math.h>
#include <map>
#include <memory>
#include <set>
#include <stdlib.h>
#include <string>
#include <time.h>
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
using drake::symbolic::Monomial;
using drake::symbolic::Polynomial;
using drake::symbolic::Variable;
using drake::symbolic::Variables;
using drake::Vector1;
using drake::VectorX;

VectorX<Monomial> GramBasis(const Polynomial& p) {
  std::pair<MatrixXi, map<Variable, int>> pair{
      build_gram_basis::PolynomialToSupport(p)};
  const MatrixXi pow{pair.first};

  // If p is a constant, we only need to return a '1' as gram basis.
  if (pair.second.empty()) {
    // TODO(FischerGundlach) According to Eigen documentation, this should take
    // a reference, i,e. Matrix (const Scalar &x). Why does it take the rvalue
    // reference?

    return Vector1<Monomial>(Monomial());

  } else {
    MatrixXi mpow{build_gram_basis::ExponentBoundPolytope(pair.first)};

    mpow = build_gram_basis::RandomPrune(
        pow, mpow, 2000);  // number of hyperplanes is copied from spotless
    mpow = build_gram_basis::DiagConsistent(pow, mpow);

    return build_gram_basis::SupportToGramBasis(mpow, pair.second);
  }
}

namespace build_gram_basis {

MatrixXi CrossTerms(MatrixXi mpow) {
  MatrixXi crossterms(NChooseK(mpow.rows(), 2), mpow.cols());
  int crossindex{0};
  for (int i = 0; i < mpow.rows() - 1; ++i) {
    for (int j = i + 1; j < mpow.rows(); ++j) {
      crossterms.row(crossindex) = mpow.row(i) + mpow.row(j);
      crossindex++;
    }
  }

  return UniqueRows(crossterms);
}

MatrixXi DiagConsistent(const MatrixXi& pow, MatrixXi mpow) {
  MatrixXi mpow_sqr{RowIntersect(2 * mpow, pow) / 2};

  auto N = mpow.rows();
  while (1) {
    MatrixXi crossterms = CrossTerms(mpow);

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

MatrixXi ExponentBoundPolytope(const MatrixXi& pow) {
  // TODO(FischerGundlach) Implement quicker combinatorial creation (dont rely
  // on the slow creation of monomialBasis).
  // TODO(FischerGundlach) Remove obviously unnecessary rows. See MATLAB
  // implementation, i.e. x_*y_ should return MatriXi(1,0)

  if (pow.cols() == 0) {
    return MatrixXi(1, 0);

  } else {
    Variables indeterminates;
    for (int i = 0; i < pow.cols(); ++i) {
      indeterminates.insert(Variable(std::to_string(i)));
    }

    const int maximum_degree_in_pow{
        (pow.rowwise().sum()).colwise().maxCoeff()[0]};
    const int maximum_degree_in_m_pow{
        static_cast<int>(std::floor(maximum_degree_in_pow / 2))};

    const VectorX<Monomial> m{drake::symbolic::MonomialBasis(
        indeterminates, maximum_degree_in_m_pow)};
    Polynomial::MapType p_map;
    for (int i = 0; i < m.size(); ++i) {
      p_map.emplace(m(i), 1);
    }

    std::pair<MatrixXi, map<Variable, int>> pair_{
        PolynomialToSupport(Polynomial(p_map))};

    return pair_.first;
  }
}

std::pair<MatrixXi, map<Variable, int>> PolynomialToSupport(
    const Polynomial& p) {
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

MatrixXi RandomPrune(const MatrixXi& pow, MatrixXi mpow,
                     const int num_hyper_planes) {
  // rows: number of rows (length of colums) is equal to number of monomials
  // cols: number of coloms (length of rows) are equal to number variables
  // => every row represents a monomial/ point in the support

  // TODO(FischerGundlach) This is in an internal namespace, aka do I need the
  // DRAKE_ASSERT?
  // mpow.cols() != 0 otherwise, the monomials are in 0 indeterminates
  // mpow.rows() != 0 otherwise there are no monomials/ points in the support
  DRAKE_ASSERT(pow.rows() == mpow.rows() && mpow.cols() != 0 &&
               mpow.rows() != 0);

  VectorXi w{mpow.cols()};
  VectorXi y{mpow.rows()};

  MatrixXi new_mpow(mpow.rows(), mpow.cols());

  double lower_bound, upper_bound;

  for (int i = 0; i < num_hyper_planes; ++i) {
    unsigned int seed{static_cast<unsigned int>(time(NULL)+i)};
    for (int j = 0; j < w.rows(); ++j) {
      w(j) = (rand_r(&seed) % 100) - 50;
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
