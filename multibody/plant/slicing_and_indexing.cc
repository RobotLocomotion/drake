#include "drake/multibody/plant/slicing_and_indexing.h"

#include <set>

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
    (&SelectRowsCols<T>, &SelectRows<T>, &SelectCols<T>, &ExpandRows<T>))

}  // namespace internal
}  // namespace multibody
}  // namespace drake
