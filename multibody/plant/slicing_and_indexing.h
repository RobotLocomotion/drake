#pragma once

#include <vector>

#include "drake/common/eigen_types.h"

// TODO(rpoyner-tri): the Select*() and Expand*() functions below can be
// removed and replaced with arbitrary indexing once Eigen 3.4 is our minimum
// required version.

namespace drake {
namespace multibody {
namespace internal {

// Valid index arrays are no larger than max_size, are sorted, and contain
// entries in the range [0, max_size).
void DemandIndicesValid(const std::vector<int>& indices, int max_size);

// Make a new, possibly smaller square matrix from square matrix @p M, by
// selecting the rows and columns indexed by @p indices.
// @pre indices.size() <= M.rows() (or M.cols() since M is a square matrix).
// @pre indices argument is valid according to DemandIndicesValid().
template <typename T>
MatrixX<T> SelectRowsCols(const MatrixX<T>& M, const std::vector<int>& indices);

// Make a new, possibly smaller matrix from matrix M, by selecting the columns
// indexed by @p indices.
// @pre indices.size() <= M.cols().
// @pre indices argument is valid according to DemandIndicesValid().
template <typename T>
MatrixX<T> SelectCols(const MatrixX<T>& M, const std::vector<int>& indices);

// Make a new, possibly smaller vector from vector @p v, by selecting the rows
// indexed by @p indices.
// @pre indices.size() <= v.size().
// @pre indices argument is valid according to DemandIndicesValid().
template <typename T>
VectorX<T> SelectRows(const VectorX<T>& v, const std::vector<int>& indices);

// Make a new, possibly larger vector of size @p rows_out, by copying rows
// of @p v to rows indexed by @p indices. New rows are filled with zeros.
// That is, v_expanded(indices(i)) = v(i), for i=0, v.size().
// @pre indices.size() <= rows_out.
// @pre indices must have the same size as v.
// @pre indices argument is valid according to DemandIndicesValid().
template <typename T>
VectorX<T> ExpandRows(const VectorX<T>& v, int rows_out,
                      const std::vector<int>& indices);

template <typename T>
VectorX<T> ExpandRows(const Eigen::VectorBlock<const VectorX<T>>& v,
                      int rows_out, const std::vector<int>& indices);

}  // namespace internal
}  // namespace multibody
}  // namespace drake
