#pragma once

#include <vector>

#include <Eigen/Dense>

namespace drake {

/// Returns true if and only if the two matrices are equal to within a certain
/// absolute elementwise @p tolerance.  Special values (infinities, NaN, etc.)
/// do not compare as equal elements.
template <typename DerivedA, typename DerivedB>
bool is_approx_equal_abstol(const Eigen::MatrixBase<DerivedA>& m1,
                            const Eigen::MatrixBase<DerivedB>& m2,
                            double tolerance) {
  return (
      (m1.rows() == m2.rows()) &&
      (m1.cols() == m2.cols()) &&
      ((m1 - m2).template lpNorm<Eigen::Infinity>() <= tolerance));
}

/// Returns true if and only if a simple greedy search reveals a permutation
/// of the columns of m2 to make the matrix equal to m1 to within a certain
/// absolute elementwise @p tolerance. E.g., there exists a P such that
/// <pre>
///    forall i,j,  |m1 - m2*P|_{i,j} <= tolerance
///    where P is a permutation matrix:
///       P(i,j)={0,1}, sum_i P(i,j)=1, sum_j P(i,j)=1.
/// </pre>
/// Note: Returns false for matrices of different sizes.
/// Note: The current implementation is O(n^2) in the number of columns.
/// Note: In marginal cases (with similar but not identical columns) this
/// algorithm can fail to find a permutation P even if it exists because it
/// accepts the first column match (m1(i),m2(j)) and removes m2(j) from the
/// pool. It is possible that other columns of m2 would also match m1(i) but
/// that m2(j) is the only match possible for a later column of m1.
template <typename DerivedA, typename DerivedB>
bool IsApproxEqualAbsTolWithPermutedColumns(
    const Eigen::MatrixBase<DerivedA>& m1,
    const Eigen::MatrixBase<DerivedB>& m2, double tolerance) {
  if ((m1.cols() != m2.cols()) || (m1.rows() != m2.rows())) return false;

  std::vector<bool> available(m2.cols());
  for (int i = 0; i < m2.cols(); i++) available[i] = true;

  for (int i = 0; i < m1.cols(); i++) {
    bool found_match = false;
    for (int j = 0; j < m2.cols(); j++) {
      if (available[j] &&
          is_approx_equal_abstol(m1.col(i), m2.col(j), tolerance)) {
        found_match = true;
        available[j] = false;
        break;
      }
    }
    if (!found_match) return false;
  }
  return true;
}


}  // namespace drake
