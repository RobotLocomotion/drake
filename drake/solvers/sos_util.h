#pragma once

#include <map>
#include <set>
#include <utility>

#include <Eigen/Core>

#include "drake/common/symbolic.h"

namespace drake {
namespace solvers {

/** Returns a ConstructGramBasis for the polynomial @p. This gram basis is
 preprocessed. @see
 RandomlyPruneSupport and @see CheckDiagConsistency. */
VectorX<symbolic::Monomial> ConstructGramBasis(const symbolic::Polynomial& p);

namespace build_gram_basis {

/** Returns 'crossterms' of the rows in @p mpow, i.e. returns row_i+row_j for
 all i
 != j.
 The size of @param mpow is dictated as following:
 the number of rows (length of colums) is equal to number of monomials and
 the number of coloms (length of rows) are equal to number of variables.
 Therefore, every row represents a monomial/point in the support. */
Eigen::MatrixXi GenerateCrossTerms(const Eigen::MatrixXi& mpow);

/** Checks diagonal consistency as in Lofberg, Johan. "Pre-and post-processing
 sum-of-squares programs in practice." IEEE Transactions on Automatic Control
 54.5 (2009): 1007-1011.
 This implementation is based on the MATLAB version
 https://github.com/frankpermenter/spotless/blob/master/util/spot_build_gram_basis.m
 The size of @param pow and @param mpow are dictated as following:
 the number of rows (length of colums) is equal to number of monomials and
 the number of coloms (length of rows) are equal to number of variables.
 Therefore, every row represents a monomial/point in the support. */
Eigen::MatrixXi CheckDiagonalConsistency(const Eigen::MatrixXi& pow,
                                         Eigen::MatrixXi mpow);

/** Returns a rough outer approximation of the needed monomial basis to
 represent
 the polynomial(pow) as SOS.
 This implementation is based on the MATLAB version
 https://github.com/spot-toolbox/spotless/blob/master/util/spot_exponent_bound_polytope.m
 The size of @param pow is dictated as following:
 the number of rows (length of columns) is equal to number of monomials and
 the number of columns (length of rows) are equal to number of variables.
 Therefore, every row represents a monomial/point in the support. */
Eigen::MatrixXi GenerateExponentBoundPolytope(const Eigen::MatrixXi& pow);

/** Returns a pair of the support of a polynomial( see: Lofberg, Johan. "Pre-and
 post-processing sum-of-squares programs in practice." IEEE Transactions on
 Automatic Control 54.5 (2009): 1007-1011.) and a mapping from variables to
 column in support.
 The size of the return Eigen::MatrixXi is dictated as following:
 the number of rows (length of colums) is equal to number of monomials and
 the number of coloms (length of rows) are equal to number of variables.
 Therefore, every row represents a monomial in @param p/ point in the support.
 */

std::pair<Eigen::MatrixXi, std::map<symbolic::Variable, int>>
PolynomialToSupport(const symbolic::Polynomial& p);

/** Phase 1 in "Kojima, Masakazu, Sunyoung Kim, and Hayato Waki. "Sparsity in
 sums of squares of polynomials." Mathematical Programming 103.1 (2005):
 45-62."

 pow is the support of the polynomial p. @see PolynomialToSupport
 mpow is an outer approximation of pow. @see GenerateExponentBoundPolytope
 The size of @param pow and @param mpow are dictated as following:
 the number of rows (length of colums) is equal to number of monomials and
 the number of coloms (length of rows) are equal to number of variables.
 Therefore, every row represents a monomial/point in the support.
 @param num_hyper_planes The amount of planes drawn to seperate @param pow and
 points in the support given by @param mpow.
 Default number of hyperplanes is copied from spotless.
 */
Eigen::MatrixXi RandomlyPruneSupport(const Eigen::MatrixXi& pow,
                                     Eigen::MatrixXi mpow,
                                     int num_hyper_planes = 2000,
                                     int seed = 9390);

/** Returns a vector filled with monomials which represent rows of support. This
 function reverses @see PolynomialToSupport*/
drake::VectorX<symbolic::Monomial> SupportToGramBasis(
    const Eigen::MatrixXi& mpow,
    const std::map<symbolic::Variable, int>& variable_to_position_map);

// From here in declaration of helper functions.

/** Removes rows of matrix_to_change, which are given by the entries of
 rows_to_remove.*/
Eigen::MatrixXi RemoveRows(const Eigen::MatrixXi& mat,
                           const std::set<int>& rows_to_remove);

/** Returns rows of mat1 which intersect rows of mat2*/
Eigen::MatrixXi RowIntersect(const Eigen::MatrixXi& mat1,
                             const Eigen::MatrixXi& mat2);

/** Returns the unique Rows of mat*/
Eigen::MatrixXi UniqueRows(const Eigen::MatrixXi& mat);

}  // namespace build_gram_basis
}  // namespace solvers
}  // namespace drake
