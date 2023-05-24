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
  std::vector<int> selected_indices;
  selected_indices.reserve(M.rows() - indices.size());

  int excluded_index = 0;
  for (int i = 0; i < M.rows(); ++i) {
    if (i == indices[excluded_index]) {
      ++excluded_index;
    } else {
      selected_indices.push_back(i);
    }
  }
  DRAKE_ASSERT(selected_indices.size() == M.rows() - indices.size());
  return SelectRowsCols(M, selected_indices);
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
contact_solvers::internal::MatrixBlock<T> SelectCols(
    const contact_solvers::internal::MatrixBlock<T>& M,
    const std::vector<int>& indices) {
  if (M.is_dense()) {
    return contact_solvers::internal::MatrixBlock<T>(
        std::move(SelectCols(M.MakeDenseMatrix(), indices)));
  } else {
    throw std::runtime_error(
        "SelectCols only supports dense MatrixBlock arguments.");
  }
}

template <typename T>
MatrixX<T> ExcludeCols(const MatrixX<T>& M, const std::vector<int>& indices) {
  DRAKE_ASSERT_VOID(DemandIndicesValid(indices, M.cols()));
  if (indices.size() == 0) {
    return M;
  }
  std::vector<int> selected_indices;
  selected_indices.reserve(M.cols() - indices.size());

  int excluded_index = 0;
  for (int i = 0; i < M.cols(); ++i) {
    if (i == indices[excluded_index]) {
      ++excluded_index;
    } else {
      selected_indices.push_back(i);
    }
  }
  DRAKE_ASSERT(selected_indices.size() == M.cols() - indices.size());
  return SelectCols(M, selected_indices);
}

template <typename T>
contact_solvers::internal::MatrixBlock<T> ExcludeCols(
    const contact_solvers::internal::MatrixBlock<T>& M,
    const std::vector<int>& indices) {
  if (M.is_dense()) {
    return contact_solvers::internal::MatrixBlock<T>(
        std::move(ExcludeCols(M.MakeDenseMatrix(), indices)));
  } else {
    throw std::runtime_error(
        "ExcludeColsCols only supports dense MatrixBlock arguments.");
  }
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
    if (exclude_index >= static_cast<int>(indices.size()) ||
        i < indices[exclude_index]) {
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

template <typename T>
VectorX<T> ExpandRows(const Eigen::VectorBlock<const VectorX<T>>& v,
                      int rows_out, const std::vector<int>& indices) {
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
     static_cast<MatrixX<T> (*)(const MatrixX<T>&, const std::vector<int>&)>(
         &SelectCols),
     static_cast<contact_solvers::internal::MatrixBlock<T> (*)(
         const contact_solvers::internal::MatrixBlock<T>&,
         const std::vector<int>&)>(&SelectCols),
     static_cast<MatrixX<T> (*)(const MatrixX<T>&, const std::vector<int>&)>(
         &ExcludeCols),
     static_cast<contact_solvers::internal::MatrixBlock<T> (*)(
         const contact_solvers::internal::MatrixBlock<T>&,
         const std::vector<int>&)>(&ExcludeCols),
     static_cast<VectorX<T> (*)(const Eigen::VectorBlock<const VectorX<T>>&,
                                int, const std::vector<int>&)>(&ExpandRows),
     static_cast<VectorX<T> (*)(const VectorX<T>&, int,
                                const std::vector<int>&)>(&ExpandRows)))

}  // namespace internal
}  // namespace multibody
}  // namespace drake
