#pragma once

#include <numeric>
#include <utility>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace pooled_sap {

// Differentiate between fixed size (e.g. Matrix3d) and dynamic size (e.g.
// MatrixXd) Eigen types.
template <typename Derived>
struct is_fixed_size
    : std::bool_constant<is_eigen_type<Derived>::value &&
                         Derived::SizeAtCompileTime != Eigen::Dynamic> {};
template <typename EigenType>
constexpr bool is_fixed_size_v = is_fixed_size<EigenType>::value;

// Detect fixed-size Eigen vectors, e.g. Vector3d.
template <typename T>
constexpr bool is_fixed_size_vector_v =
    is_eigen_vector<T>::value && is_fixed_size_v<T>;

// Detect dynamic-size Eigen vectors, e.g. VectorXd.
template <typename T>
constexpr bool is_dynamic_size_vector_v =
    is_eigen_vector<T>::value && !is_fixed_size_v<T>;

// Detect matrices with fixed number of rows, e.g. Matrix6Xd.
template <typename T>
constexpr bool has_fixed_size_rows_v =
    is_eigen_type<T>::value && T::RowsAtCompileTime != Eigen::Dynamic;

// Detect matrices with fixed number of columns, e.g. MatrixX4d. This includes
// vectors (e.g. VectorXd).
template <typename T>
constexpr bool has_fixed_size_cols_v =
    is_eigen_type<T>::value && T::ColsAtCompileTime != Eigen::Dynamic;

/**
 * Contiguous storage for a pool of fixed-size Eigen objects.
 *
 * This is essentially a wrapper around std::vector<EigenType>, where EigenType
 * is something like Matrix3d, Vector4d, etc.
 */
template <typename EigenType>
struct FixedSizeStorage {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(FixedSizeStorage);

  static_assert(EigenType::SizeAtCompileTime != Eigen::Dynamic,
                "Only for fixed-size Eigen types.");

  using Scalar = typename EigenType::Scalar;
  using ElementView = EigenType&;
  using ConstElementView = const EigenType&;

  // Contiguous storage for all Eigen objects in the pool.
  std::vector<EigenType> data_;

  FixedSizeStorage() = default;

  // Set the size to zero, but keep the allocated memory.
  void Clear() { data_.clear(); }

  // Resize to store `num_elements`, allocating additional memory if needed.
  void Resize(int num_elements) { data_.resize(num_elements); }

  // Sets all elements in the pool to zero.
  void SetZero() {
    Eigen::Map<VectorX<Scalar>>(data_.data()->data(),
                                data_.size() * EigenType::SizeAtCompileTime)
        .setZero();
  }

  // The number of elements in the pool.
  int size() const { return data_.size(); }

  // Access the i-th element in the pool.
  ConstElementView at(int i) const { return data_.at(i); }
  ElementView at(int i) { return data_.at(i); }
};

/**
 * Contiguous storage for a pool of dynamic-size Eigen objects.
 *
 * Each element in the pool can have a different size, but they must have the
 * same type (e.g. MatrixXd, VectorXd, VectorX<AutoDiffXd>, etc).
 */
template <typename EigenType>
struct DynamicSizeStorage {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DynamicSizeStorage);

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

  // Index into data_ for the next Eigen element.
  int next_data_index_{0};

  // Contiguous storage for all Eigen objects in the pool.
  std::vector<Scalar> data_;

  // Properly sized maps to each element in the pool.
  std::vector<ElementData> blocks_;

  DynamicSizeStorage() = default;

  // Resize the pool to store MatrixX elements of the specified sizes.
  // N.B. rows (cols) are ignored if the rows (cols) of EigenType are fixed at
  // compile time.
  void Resize(const std::vector<int>& rows, const std::vector<int>& cols) {
    static_assert(!is_fixed_size_v<EigenType>);
    DRAKE_ASSERT(rows.size() == cols.size());
    Clear();
    int total_size = 0;
    for (int i = 0; i < ssize(rows); ++i) {
      total_size += rows[i] * cols[i];
    }
    data_.reserve(total_size);
    blocks_.reserve(ssize(rows));
    for (int i = 0; i < ssize(rows); ++i) {
      const int r = EigenType::RowsAtCompileTime >= 0
                        ? EigenType::RowsAtCompileTime
                        : rows[i];
      const int c = EigenType::ColsAtCompileTime >= 0
                        ? EigenType::ColsAtCompileTime
                        : cols[i];
      Add(r, c);
    }
  }

  // Resize the pool to store `num_elements` elements, each of size (rows x
  // cols).
  void Resize(int num_elements, int rows, int cols) {
    static_assert(!is_fixed_size_v<EigenType>);
    const int r =
        EigenType::RowsAtCompileTime >= 0 ? EigenType::RowsAtCompileTime : rows;
    const int c =
        EigenType::ColsAtCompileTime >= 0 ? EigenType::ColsAtCompileTime : cols;
    Clear();
    data_.reserve(num_elements * r * c);
    blocks_.reserve(num_elements);
    for (int i = 0; i < num_elements; ++i) {
      Add(r, c);
    }
  }

  // Set the size to zero, but keep any allocated memory.
  void Clear() {
    next_data_index_ = 0;
    data_.clear();
    blocks_.clear();
  }

  // Append a new element of the specified size to the end of the pool.
  void Add(int rows, int cols) {
    const int size = rows * cols;
    data_.resize(data_.size() + size);
    blocks_.push_back({next_data_index_, rows, cols});
    next_data_index_ += size;
  }

  // Sets all elements in the pool to zero.
  void SetZero() {
    Eigen::Map<VectorX<Scalar>>(data_.data(), data_.size()).setZero();
  }

  // The number of elements in the pool.
  int size() const { return blocks_.size(); }

  // Access the i-th element in the pool.
  ConstElementView at(int i) const {
    return ConstElementView(&data_.at(blocks_[i].index), blocks_[i].rows,
                            blocks_[i].cols);
  }
  ElementView at(int i) {
    return ElementView(&data_.at(blocks_[i].index), blocks_[i].rows,
                       blocks_[i].cols);
  }
};

// Choose fixed-size storage (e.g. Matrix3d).
template <typename EigenType, int size_at_compile_time>
struct StorageSelector {
  using Storage = FixedSizeStorage<EigenType>;
};

// Choose dynamic-size storage (e.g. MatrixXd).
template <typename EigenType>
struct StorageSelector<EigenType, Eigen::Dynamic> {
  using Storage = DynamicSizeStorage<EigenType>;
};

/**
 * A replacement for std::vector<MatrixX<T>> with a contiguous memory layout.
 *
 * @tparam EigenType The type of the Eigen elements. E.g. MatrixXd,
 *         Vector3d, etc.
 * @pre EigenType must be an Eigen type derived from Eigen::MatrixBase.
 */
template <typename EigenType>
class EigenPool {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(EigenPool);

  static_assert(is_eigen_type<EigenType>::value, "Must be an Eigen type.");

  using Scalar = typename EigenType::Scalar;
  using Storage =
      typename StorageSelector<EigenType,
                               EigenType::SizeAtCompileTime>::Storage;
  using ElementView = typename Storage::ElementView;
  using ConstElementView = typename Storage::ConstElementView;

  // Default constructor for an empty pool.
  EigenPool() = default;

  // Resize methods allocate memory only if the current capacity is
  // insufficient, and do not reduce the capacity.

  // Resize a pool of fixed-size elements (e.g. Matrix3d).
  void Resize(int num_elements)
    requires is_fixed_size_v<EigenType>
  {  // NOLINT(whitespace/braces)
    storage_.Resize(num_elements);
  }

  // Resize a pool of dynamic-size matrices (e.g. MatrixXd).
  // N.B. rows (cols) are ignored if the rows (cols) of EigenType are fixed at
  // compile time.
  void Resize(const std::vector<int>& rows, const std::vector<int>& cols)
    requires(!is_fixed_size_v<EigenType>)
  {  // NOLINT(whitespace/braces)
    storage_.Resize(rows, cols);
  }

  // Resize a pool of matrices with a fixed number of rows or columns, e.g.
  // Matrix6Xd or VectorXd.
  void Resize(const std::vector<int>& sizes)
    requires(has_fixed_size_rows_v<EigenType> ||
             has_fixed_size_cols_v<EigenType>)
  {  // NOLINT(whitespace/braces)
    storage_.Resize(sizes, sizes);
  }

  // Resize a pool of dynamic-size matrices (e.g. MatrixXd), where all elements
  // have the same size.
  // N.B. rows (cols) are ignored if the rows (cols) of EigenType are fixed at
  // compile time.
  void Resize(int num_elements, int rows, int cols)
    requires(!is_fixed_size_v<EigenType>)
  {  // NOLINT(whitespace/braces)
    storage_.Resize(num_elements, rows, cols);
  }

  // Clears data. Capacity is not changed, and thus memory is not freed.
  void Clear() { storage_.Clear(); }

  // Zeroes out all elements in the pool.
  void SetZero() { storage_.SetZero(); }

  // Returns the number of elements in the pool.
  int size() const { return storage_.size(); }

  // Const access to the i-th element.
  const ConstElementView operator[](int i) const {
    DRAKE_DEMAND(0 <= i && i < size());
    return storage_.at(i);
  }

  // Non-const access to the i-th element.
  ElementView operator[](int i) {
    DRAKE_DEMAND(0 <= i && i < size());
    return storage_.at(i);
  }

 private:
  // The underlying data, FixedSizeStorage or DynamicSizeStorage.
  Storage storage_;
};

}  // namespace pooled_sap
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
