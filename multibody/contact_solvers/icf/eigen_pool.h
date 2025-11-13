#pragma once

#include <numeric>
#include <span>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace icf {
namespace internal {

/* Contiguous storage for a pool of fixed-size Eigen objects.

This is essentially a wrapper around std::vector<EigenType>, where EigenType
is something like Matrix3d, Vector4d, etc. */
template <typename EigenType>
struct FixedSizeStorage {
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FixedSizeStorage);

  static_assert(EigenType::SizeAtCompileTime != Eigen::Dynamic,
                "Only for fixed-size Eigen types.");

  using Scalar = typename EigenType::Scalar;
  using ElementView = EigenType&;
  using ConstElementView = const EigenType&;

  /* Contiguous storage for all Eigen objects in the pool. */
  std::vector<EigenType> data_;

  FixedSizeStorage() = default;

  /* Set the size to zero, but keep the allocated memory. */
  void Clear() { data_.clear(); }

  /* Resizes to store `num_elements`, allocating additional memory if needed.
  @pre `rows` and `cols` must match the compile-time size of the EigenType. */
  void Resize(int num_elements, int /* rows */, int /* cols */) {
    data_.resize(num_elements);
  }

  /* Sets all elements in the pool to zero. */
  void SetZero() {
    Eigen::Map<VectorX<Scalar>>(data_.data()->data(),
                                data_.size() * EigenType::SizeAtCompileTime)
        .setZero();
  }

  /* The number of elements in the pool. */
  int size() const { return data_.size(); }

  /* Access the i-th element in the pool. */
  ConstElementView at(int i) const { return data_.at(i); }
  ElementView at(int i) { return data_.at(i); }
};

/* Contiguous storage for a pool of dynamic-size Eigen objects.

Each element in the pool can have a different size, but they must have the
same type (e.g. MatrixXd, VectorXd, VectorX<AutoDiffXd>, etc). */
template <typename EigenType>
struct DynamicSizeStorage {
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DynamicSizeStorage);

  static_assert(EigenType::SizeAtCompileTime == Eigen::Dynamic,
                "Only for dynamics-size Eigen types.");

  using Scalar = typename EigenType::Scalar;
  using ElementView = Eigen::Map<EigenType>;
  using ConstElementView = Eigen::Map<const EigenType>;

  struct ElementData {
    int index{0};  // index into Storage::data_.
    int rows{0};   // Number of rows.
    int cols{0};   // Number of columns.
  };

  /* Index into data_ for the next Eigen element. */
  int next_data_index_{0};

  /* Contiguous storage for all Eigen objects in the pool. */
  std::vector<Scalar> data_;

  /* Properly sized maps to each element in the pool. */
  std::vector<ElementData> blocks_;

  DynamicSizeStorage() = default;

  /* Resizes to store `num_elements` elements, each of size (rows x cols).
  @pre If either RowsAtCompileTime or ColsAtCompileTime is not Eigen::Dynamic,
  then the respective `rows` or `cols` argument must match the compile-time
  size. */
  void Resize(int num_elements, int rows, int cols) {
    Clear();
    data_.reserve(num_elements * rows * cols);
    blocks_.reserve(num_elements);
    for (int i = 0; i < num_elements; ++i) {
      Add(rows, cols);
    }
  }

  /* Resizes to store `num_elements` elements with the given sizes.
  @pre If either RowsAtCompileTime or ColsAtCompileTime is not Eigen::Dynamic,
  then the respective `rows` or `cols` argument must be empty. */
  void Resize(int num_elements, std::span<const int> rows,
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

  /* Set the size to zero, but keep any allocated memory. */
  void Clear() {
    next_data_index_ = 0;
    data_.clear();
    blocks_.clear();
  }

  /* Append a new element of the specified size to the end of the pool. */
  void Add(int rows, int cols) {
    const int size = rows * cols;
    data_.resize(data_.size() + size);
    blocks_.push_back({next_data_index_, rows, cols});
    next_data_index_ += size;
  }

  /* Sets all elements in the pool to zero. */
  void SetZero() {
    Eigen::Map<VectorX<Scalar>>(data_.data(), data_.size()).setZero();
  }

  /* The number of elements in the pool. */
  int size() const { return blocks_.size(); }

  /* Access the i-th element in the pool. */
  ConstElementView at(int i) const {
    return ConstElementView(&data_.at(blocks_[i].index), blocks_[i].rows,
                            blocks_[i].cols);
  }
  ElementView at(int i) {
    return ElementView(&data_.at(blocks_[i].index), blocks_[i].rows,
                       blocks_[i].cols);
  }
};

/* Choose fixed-size storage (e.g. Matrix3d). */
template <typename EigenType, int size_at_compile_time>
struct StorageSelector {
  using Storage = FixedSizeStorage<EigenType>;
};

/* Choose dynamic-size storage (e.g. MatrixXd). */
template <typename EigenType>
struct StorageSelector<EigenType, Eigen::Dynamic> {
  using Storage = DynamicSizeStorage<EigenType>;
};

/* A replacement for std::vector<MatrixX<T>> with a contiguous memory layout.

@tparam EigenType The type of the Eigen elements, e.g. MatrixXd, Vector3d, etc.
                  It must be an Eigen type derived from Eigen::MatrixBase. */
template <typename EigenType>
class EigenPool {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(EigenPool);

  static_assert(is_eigen_type<EigenType>::value, "Must be an Eigen type.");

  using Scalar = typename EigenType::Scalar;
  using Storage =
      typename StorageSelector<EigenType,
                               EigenType::SizeAtCompileTime>::Storage;
  using ElementView = typename Storage::ElementView;
  using ConstElementView = typename Storage::ConstElementView;

  /* Default constructor for an empty pool. */
  EigenPool() = default;

  // Resize methods allocate memory only if the current capacity is
  // insufficient, and do not reduce the capacity.

  /* Resizes a pool using a homogenous size for all elements. If either or both
  of RowsAtCompileTime or ColsAtCompileTime are not Eigen::Dynamic, then the
  respective `rows` or `cols` argument must match the compile-time size. */
  void Resize(int num_elements, int rows, int cols) {
    DRAKE_DEMAND(EigenType::RowsAtCompileTime == Eigen::Dynamic ||
                 rows == EigenType::RowsAtCompileTime);
    DRAKE_DEMAND(EigenType::ColsAtCompileTime == Eigen::Dynamic ||
                 cols == EigenType::ColsAtCompileTime);
    storage_.Resize(num_elements, rows, cols);
  }

  /* Resizes a pool using heterogenous sizes for the elements. If either or both
  of RowsAtCompileTime or ColsAtCompileTime are not Eigen::Dynamic, then the
  respective `rows` or `cols` argument must be empty. */
  void Resize(int num_elements, std::span<const int> rows,
              std::span<const int> cols = {}) {
    static_assert(EigenType::SizeAtCompileTime == Eigen::Dynamic);
    DRAKE_DEMAND(EigenType::RowsAtCompileTime == Eigen::Dynamic ||
                 rows.empty());
    DRAKE_DEMAND(EigenType::ColsAtCompileTime == Eigen::Dynamic ||
                 cols.empty());
    storage_.Resize(num_elements, rows, cols);
  }

  /* Clears data. Capacity is not changed, and thus memory is not freed. */
  void Clear() { storage_.Clear(); }

  /* Zeroes out all elements in the pool. */
  void SetZero() { storage_.SetZero(); }

  /* Returns the number of elements in the pool. */
  int size() const { return storage_.size(); }

  /* Const access to the i-th element. */
  const ConstElementView operator[](int i) const {
    DRAKE_DEMAND(0 <= i && i < size());
    return storage_.at(i);
  }

  /* Non-const access to the i-th element. */
  ElementView operator[](int i) {
    DRAKE_DEMAND(0 <= i && i < size());
    return storage_.at(i);
  }

 private:
  /* The underlying data, FixedSizeStorage or DynamicSizeStorage. */
  Storage storage_;
};

}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
