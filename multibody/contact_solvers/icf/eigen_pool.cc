#include "drake/multibody/contact_solvers/icf/eigen_pool.h"

#include <numeric>

#include "drake/common/autodiff.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace icf {
namespace internal {

template <typename EigenType>
EigenPoolFixedSizeStorage<EigenType>::EigenPoolFixedSizeStorage() = default;

template <typename EigenType>
EigenPoolFixedSizeStorage<EigenType>::~EigenPoolFixedSizeStorage() = default;

template <typename EigenType>
void EigenPoolFixedSizeStorage<EigenType>::Resize(int num_elements,
                                                  int /* rows */,
                                                  int /* cols */) {
  data_.resize(num_elements);
}

template <typename EigenType>
void EigenPoolFixedSizeStorage<EigenType>::Clear() {
  data_.clear();
}

template <typename EigenType>
void EigenPoolFixedSizeStorage<EigenType>::SetZero() {
  Scalar* const first_scalar = data_.data()->data();
  const int num_scalars = data_.size() * EigenType::SizeAtCompileTime;
  Eigen::Map<VectorX<Scalar>>(first_scalar, num_scalars).setZero();
}

template <typename EigenType>
EigenPoolDynamicSizeStorage<EigenType>::EigenPoolDynamicSizeStorage() = default;

template <typename EigenType>
EigenPoolDynamicSizeStorage<EigenType>::~EigenPoolDynamicSizeStorage() =
    default;

template <typename EigenType>
void EigenPoolDynamicSizeStorage<EigenType>::Resize(int num_elements, int rows,
                                                    int cols) {
  Clear();
  data_.reserve(num_elements * rows * cols);
  blocks_.reserve(num_elements);
  for (int i = 0; i < num_elements; ++i) {
    Add(rows, cols);
  }
}

template <typename EigenType>
void EigenPoolDynamicSizeStorage<EigenType>::Resize(int num_elements,
                                                    std::span<const int> rows,
                                                    std::span<const int> cols) {
  Clear();
  constexpr int fixed_rows = EigenType::RowsAtCompileTime;
  constexpr int fixed_cols = EigenType::ColsAtCompileTime;
  static_assert(fixed_rows == Eigen::Dynamic || fixed_cols == Eigen::Dynamic);
  if constexpr (fixed_rows >= 0 || fixed_cols >= 0) {
    // Only dynamic in one dimension.
    const int fixed_dim = (fixed_rows >= 0) ? fixed_rows : fixed_cols;
    const std::span<const int>& dyn_dims = (fixed_rows >= 0) ? cols : rows;
    DRAKE_DEMAND(ssize(dyn_dims) == num_elements);
    const int total_size =
        fixed_dim * std::accumulate(dyn_dims.begin(), dyn_dims.end(), 0);
    data_.reserve(total_size);
    blocks_.reserve(num_elements);
    for (int i = 0; i < num_elements; ++i) {
      const int r = fixed_rows >= 0 ? fixed_rows : rows[i];
      const int c = fixed_cols >= 0 ? fixed_cols : cols[i];
      Add(r, c);
    }
  } else {
    // Fully dynamic.
    DRAKE_DEMAND(ssize(rows) == num_elements);
    DRAKE_DEMAND(ssize(cols) == num_elements);
    int total_size = 0;
    for (int i = 0; i < num_elements; ++i) {
      total_size += rows[i] * cols[i];
    }
    data_.reserve(total_size);
    blocks_.reserve(num_elements);
    for (int i = 0; i < num_elements; ++i) {
      Add(rows[i], cols[i]);
    }
  }
}

template <typename EigenType>
void EigenPoolDynamicSizeStorage<EigenType>::SetZero() {
  Eigen::Map<VectorX<Scalar>>(data_.data(), data_.size()).setZero();
}

template <typename EigenType>
void EigenPoolDynamicSizeStorage<EigenType>::Clear() {
  data_.clear();
  blocks_.clear();
}

template <typename EigenType>
void EigenPoolDynamicSizeStorage<EigenType>::Add(int rows, int cols) {
  const int size = rows * cols;
  blocks_.push_back({static_cast<int>(ssize(data_)), rows, cols});
  data_.resize(data_.size() + size);
}

template <typename EigenType>
EigenPool<EigenType>::EigenPool() = default;

template <typename EigenType>
EigenPool<EigenType>::~EigenPool() = default;

// Explicitly instantiate all types listed in the header (in order).

// Matrix3
template class EigenPool<Matrix3<double>>;
template class EigenPoolFixedSizeStorage<Matrix3<double>>;
template class EigenPool<Matrix3<AutoDiffXd>>;
template class EigenPoolFixedSizeStorage<Matrix3<AutoDiffXd>>;

// Matrix3X
template class EigenPool<Matrix3X<double>>;
template class EigenPoolDynamicSizeStorage<Matrix3X<double>>;
template class EigenPool<Matrix3X<AutoDiffXd>>;
template class EigenPoolDynamicSizeStorage<Matrix3X<AutoDiffXd>>;

// Matrix6
template class EigenPool<Matrix6<double>>;
template class EigenPoolFixedSizeStorage<Matrix6<double>>;
template class EigenPool<Matrix6<AutoDiffXd>>;
template class EigenPoolFixedSizeStorage<Matrix6<AutoDiffXd>>;

// Matrix6X
template class EigenPool<Matrix6X<double>>;
template class EigenPoolDynamicSizeStorage<Matrix6X<double>>;
template class EigenPool<Matrix6X<AutoDiffXd>>;
template class EigenPoolDynamicSizeStorage<Matrix6X<AutoDiffXd>>;

// MatrixX
template class EigenPool<MatrixX<double>>;
template class EigenPoolDynamicSizeStorage<MatrixX<double>>;
template class EigenPool<MatrixX<AutoDiffXd>>;
template class EigenPoolDynamicSizeStorage<MatrixX<AutoDiffXd>>;

// Vector3
template class EigenPool<Vector3<double>>;
template class EigenPoolFixedSizeStorage<Vector3<double>>;
template class EigenPool<Vector3<AutoDiffXd>>;
template class EigenPoolFixedSizeStorage<Vector3<AutoDiffXd>>;

// Vector6
template class EigenPool<Vector6<double>>;
template class EigenPoolFixedSizeStorage<Vector6<double>>;
template class EigenPool<Vector6<AutoDiffXd>>;
template class EigenPoolFixedSizeStorage<Vector6<AutoDiffXd>>;

// VectorX
template class EigenPool<VectorX<double>>;
template class EigenPoolDynamicSizeStorage<VectorX<double>>;
template class EigenPool<VectorX<AutoDiffXd>>;
template class EigenPoolDynamicSizeStorage<VectorX<AutoDiffXd>>;

}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
