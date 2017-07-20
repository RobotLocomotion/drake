//
// Created by fischergundlach on 7/3/17.
//
#pragma once

#include <map>
#include <set>
#include <utility>

#include <Eigen/Core>

#include "drake/common/symbolic.h"

namespace drake {
namespace solvers {

// Returns a GramBasis for the poylnomial. This gram basis is preprocessed. see@
// RandomPrune and see@ DiagConsistent
drake::VectorX<symbolic::Monomial> GramBasis(const symbolic::Polynomial& p);

namespace build_gram_basis {

// Returns 'crossterms' of the rows in mpow, i.e. returns row_i+row_j for all i
// != j.
// rows: number of rows (length of colums) is equal to number of monomials
// cols: number of coloms (length of rows) are equal to number variables
// => every row represents a monomial/ point in the support
Eigen::MatrixXi CrossTerms(Eigen::MatrixXi mpow);

// Diagonal Consistency in: Lofberg, Johan. "Pre-and post-processing
// sum-of-squares programs in practice." IEEE Transactions on Automatic Control
// 54.5 (2009): 1007-1011.
// This implementation is based on the MATLAB version
// https://github.com/frankpermenter/spotless/blob/master/util/spot_build_gram_basis.m
// rows: number of rows (length of colums) is equal to number of monomials
// cols: number of coloms (length of rows) are equal to number variables
// => every row represents a monomial/ point in the support
Eigen::MatrixXi DiagConsistent(const Eigen::MatrixXi& pow,
                               Eigen::MatrixXi mpow);

// Returns a rough outer approximation of the needed monomial basis to represent
// the polynomial(pow) as SOS.
// This implementation is based on the MATLAB version
// https://github.com/spot-toolbox/spotless/blob/master/util/spot_exponent_bound_polytope.m
// rows: number of rows (length of colums) is equal to number of monomials
// cols: number of coloms (length of rows) are equal to number variables
// => every row represents a monomial/ point in the support
Eigen::MatrixXi ExponentBoundPolytope(const Eigen::MatrixXi& pow);

// Returns a pair of the support of a polynomial( see: Lofberg, Johan. "Pre-and
// post-processing sum-of-squares programs in practice." IEEE Transactions on
// Automatic Control 54.5 (2009): 1007-1011.) and a mapping from variables to
// column in support.
// rows: number of rows (length of colums) is equal to number of monomials
// cols: number of coloms (length of rows) are equal to number variables
// => every row represents a monomial/ point in the support
std::pair<Eigen::MatrixXi, std::map<symbolic::Variable, int>>
PolynomialToSupport(const symbolic::Polynomial& p);

// Phase 1 in "Kojima, Masakazu, Sunyoung Kim, and Hayato Waki. "Sparsity in
// sums of squares of polynomials." Mathematical Programming 103.1 (2005):
// 45-62."
// This implementation is based on the MATLAB version
// https://github.com/frankpermenter/spotless/blob/master/util/spot_build_gram_basis.m
// pow is the support of the polynomial p. @see PolynomialToSupport
// mpow is an outer approximation of pow. @see ExponentBoundPolytope
// rows: number of rows (length of colums) is equal to number of monomials
// cols: number of coloms (length of rows) are equal to number variables
// => every row represents a monomial/ point in the support
Eigen::MatrixXi RandomPrune(const Eigen::MatrixXi& pow, Eigen::MatrixXi mpow,
                            int num_hyper_planes);

// Returns a vector filled with monomials which represent rows of support. This
// function reverses @see PolynomialToSupport
drake::VectorX<symbolic::Monomial> SupportToGramBasis(
    const Eigen::MatrixXi& mpow,
    const std::map<symbolic::Variable, int>& variable_to_position_map);

// From here in declaration of helper functions.

// Computes "n choose k", the number of ways, disregarding order, that k objects
// can be chosen from among n objects. It is used in the CrossTerms function.
constexpr int NChooseK(int n, int k) {
  return (k == 0) ? 1 : (n * NChooseK(n - 1, k - 1)) / k;
}

// Removes rows of matrix_to_change, which are given by the entries of
// rows_to_remove.
Eigen::MatrixXi RemoveRows(const Eigen::MatrixXi& mat,
                           const std::set<int>& rows_to_remove);

// Returns rows of mat1 which intersect rows of mat2
Eigen::MatrixXi RowIntersect(const Eigen::MatrixXi& mat1,
                             const Eigen::MatrixXi& mat2);

// Returns the unique Rows of mat
Eigen::MatrixXi UniqueRows(const Eigen::MatrixXi& mat);

}  // namespace build_gram_basis
}  // namespace solvers
}  // namespace drake
