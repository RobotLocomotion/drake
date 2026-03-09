#include "drake/multibody/plant/slicing_and_indexing.h"

#include <set>
#include <utility>

#include "drake/common/default_scalars.h"
#include "drake/common/eigen_types.h"

// TODO(rpoyner-tri): the Select*() and Expand*() functions below can be
// removed and replaced with arbitrary indexing once Eigen 3.4 is our minimum
// required version.

namespace drake {
namespace multibody {
namespace internal {

void DemandIndicesValid(const std::vector<int>& indices, int max_size) {
  DRAKE_DEMAND(static_cast<int>(indices.size()) <= max_size);
  if (indices.empty()) {
    return;
  }

  // Only do the expensive check in debug builds.
  DRAKE_ASSERT(std::is_sorted(indices.begin(), indices.end()));
  DRAKE_ASSERT(std::set<int>(indices.begin(), indices.end()).size() ==
               indices.size());  // Checks there are no duplicates.
  DRAKE_DEMAND(indices[0] >= 0);
  DRAKE_DEMAND(indices[indices.size() - 1] < max_size);
}

template <typename T>
MatrixX<T> SelectRowsCols(const MatrixX<T>& M,
                          const std::vector<int>& indices) {
  DRAKE_DEMAND(M.rows() == M.cols());
  DRAKE_ASSERT_VOID(DemandIndicesValid(indices, M.rows()));
  const int selected_count = indices.size();
  if (selected_count == M.rows()) {
    return M;
  }
  MatrixX<T> result(selected_count, selected_count);

  for (int i = 0; i < result.rows(); ++i) {
    for (int j = 0; j < result.cols(); ++j) {
      result(i, j) = M(indices[i], indices[j]);
    }
  }
  return result;
}

template <typename T>
MatrixX<T> ExcludeRowsCols(const MatrixX<T>& M,
                           const std::vector<int>& indices) {
  DRAKE_DEMAND(M.rows() == M.cols());
  DRAKE_ASSERT_VOID(DemandIndicesValid(indices, M.rows()));
  if (indices.size() == 0) {
    return M;
  }

  MatrixX<T> result(M.rows() - indices.size(), M.cols() - indices.size());

  int r_index = 0;          // Keeps track of the row of result being filled.
  int r_exclude_index = 0;  // Index into indices of the next row to exclude.
  for (int i = 0; i < M.rows(); ++i) {
    // If we still have rows to exclude and are on the next row to exclude,
    // skip this row.
    if (r_exclude_index < ssize(indices) && i == indices[r_exclude_index]) {
      ++r_exclude_index;
      continue;
    }
    // Keeps track of the column of result being filled.
    int c_index = 0;
    // Index into indices of the next column to exclude.
    int c_exclude_index = 0;
    for (int j = 0; j < M.cols(); ++j) {
      // If we still have columns to exclude and are on the next columns to
      // exclude, skip this column.
      if (c_exclude_index < ssize(indices) && j == indices[c_exclude_index]) {
        ++c_exclude_index;
        continue;
      }
      result(r_index, c_index) = M(i, j);
      ++c_index;
    }
    DRAKE_ASSERT(c_index == result.cols());
    DRAKE_ASSERT(c_exclude_index == ssize(indices));
    ++r_index;
  }

  DRAKE_ASSERT(r_index == result.rows());
  DRAKE_ASSERT(r_exclude_index == ssize(indices));
  return result;
}

template <typename T>
MatrixX<T> SelectCols(const MatrixX<T>& M, const std::vector<int>& indices) {
  DRAKE_ASSERT_VOID(DemandIndicesValid(indices, M.cols()));
  const int selected_count = indices.size();
  if (selected_count == M.cols()) {
    return M;
  }
  MatrixX<T> result(M.rows(), selected_count);

  for (int i = 0; i < result.cols(); ++i) {
    result.col(i) = M.col(indices[i]);
  }
  return result;
}

template <typename T>
MatrixX<T> ExcludeCols(const MatrixX<T>& M, const std::vector<int>& indices) {
  DRAKE_ASSERT_VOID(DemandIndicesValid(indices, M.cols()));
  if (indices.size() == 0) {
    return M;
  }

  MatrixX<T> result(M.rows(), M.cols() - indices.size());

  int c_index = 0;          // Keeps track of the column of result being filled.
  int c_exclude_index = 0;  // Index into indices of the next column to exclude.
  for (int j = 0; j < M.cols(); ++j) {
    // If we've run out of columns to exclude or are not the next column to
    // exclude, include this column.
    if (c_exclude_index >= ssize(indices) || j < indices[c_exclude_index]) {
      result.col(c_index) = M.col(j);
      ++c_index;
    } else {
      ++c_exclude_index;
    }
  }

  DRAKE_ASSERT(c_index == result.cols());
  DRAKE_ASSERT(c_exclude_index == ssize(indices));
  return result;
}

template <typename T>
contact_solvers::internal::MatrixBlock<T> ExcludeCols(
    const contact_solvers::internal::MatrixBlock<T>& M,
    const std::vector<int>& indices) {
  DRAKE_THROW_UNLESS(indices.size() == 0 || M.is_dense());
  if (indices.size() == 0) {
    return M;
  }

  return contact_solvers::internal::MatrixBlock<T>(
      ExcludeCols(M.MakeDenseMatrix(), indices));
}

template <typename T>
VectorX<T> SelectRows(const VectorX<T>& v, const std::vector<int>& indices) {
  DRAKE_ASSERT_VOID(DemandIndicesValid(indices, v.size()));
  const int selected_count = indices.size();
  if (selected_count == v.rows()) {
    return v;
  }
  VectorX<T> result(selected_count);

  for (int i = 0; i < result.rows(); ++i) {
    result(i) = v(indices[i]);
  }
  return result;
}

template <typename T>
VectorX<T> ExcludeRows(const VectorX<T>& v, const std::vector<int>& indices) {
  DRAKE_ASSERT_VOID(DemandIndicesValid(indices, v.size()));
  const int selected_count = indices.size();
  if (selected_count == 0) {
    return v;
  }
  VectorX<T> result(v.rows() - selected_count);

  int r_index = 0;
  int exclude_index = 0;
  for (int i = 0; i < v.rows(); ++i) {
    if (exclude_index >= ssize(indices) || i < indices[exclude_index]) {
      result(r_index) = v(i);
      ++r_index;
    } else {
      ++exclude_index;
    }
  }
  return result;
}

template <typename T>
VectorX<T> ExpandRows(const VectorX<T>& v, int rows_out,
                      const std::vector<int>& indices) {
  DRAKE_ASSERT(static_cast<int>(indices.size()) == v.rows());
  DRAKE_ASSERT(rows_out >= v.rows());
  DRAKE_ASSERT_VOID(DemandIndicesValid(indices, rows_out));
  if (rows_out == v.rows()) {
    return v;
  }
  VectorX<T> result(rows_out);

  int index_cursor = 0;
  for (int i = 0; i < result.rows(); ++i) {
    if (index_cursor >= v.rows() || i < indices[index_cursor]) {
      result(i) = 0.;
    } else {
      result(indices[index_cursor]) = v(index_cursor);
      ++index_cursor;
    }
  }
  return result;
}

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    (&SelectRowsCols<T>, &ExcludeRowsCols<T>, &SelectRows<T>, &ExcludeRows<T>,
     &SelectCols<T>, &ExpandRows<T>,
     static_cast<MatrixX<T> (*)(const MatrixX<T>&, const std::vector<int>&)>(
         &ExcludeCols),
     static_cast<contact_solvers::internal::MatrixBlock<T> (*)(
         const contact_solvers::internal::MatrixBlock<T>&,
         const std::vector<int>&)>(&ExcludeCols)));

}  // namespace internal
}  // namespace multibody
}  // namespace drake
