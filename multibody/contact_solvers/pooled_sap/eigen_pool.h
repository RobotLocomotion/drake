#pragma once

#include <numeric>
#include <utility>
#include <vector>

// #include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace pooled_sap {

template <typename Derived>
struct is_fixed_size
    : std::bool_constant<is_eigen_type<Derived>::value &&
                         Derived::SizeAtCompileTime != Eigen::Dynamic> {};
template <typename EigenType>
constexpr bool is_fixed_size_v = is_fixed_size<EigenType>::value;

template <typename T>
constexpr bool is_fixed_size_vector_v =
    is_eigen_vector<T>::value && is_fixed_size_v<T>;

template <typename T>
constexpr bool is_dynamic_size_vector_v =
    is_eigen_vector<T>::value && !is_fixed_size_v<T>;

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

  /* Resizes pool to store VectorX elements of the specified sizes. */
  void Resize(const std::vector<int>& sizes) {
    static_assert(is_dynamic_size_vector_v<EigenType>);
    Clear();
    const int total_size = std::accumulate(sizes.begin(), sizes.end(), 0);
    data_.reserve(total_size);
    blocks_.reserve(ssize(sizes));
    for (int sz : sizes) {
      Add(sz, 1);
    }
  }

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

  void Clear() {
    next_data_index_ = 0;
    data_.clear();
    blocks_.clear();
  }

  void PushBack(const EigenType& data) { AddAndCopy(data); }

  // Capcity to store Eigen elements.
  int elements_capacity() const { return blocks_.capacity(); }

  // Capacity to store scalar entries across all elements.
  int scalars_capacity() const { return data_.capacity(); }

  // Adds new element and returns mutable view to it.
  ElementView Add(int rows, int cols) {
    const int index = size();
    const int size = rows * cols;
    data_.resize(data_.size() + size);
    blocks_.push_back({next_data_index_, rows, cols});
    next_data_index_ += size;
    return at(index);
  }

  // Adds new element and copies `data` into it.
  // @returns index to the new element.
  ElementView AddAndCopy(const EigenType& data) {
    return Add(data.rows(), data.cols()) = data;
    // return at(size());
  }

  void SetZero() {
    Eigen::Map<VectorX<Scalar>>(data_.data(), data_.size()).setZero();
  }

  int size() const { return blocks_.size(); }
  ConstElementView at(int i) const {
    return ConstElementView(&data_.at(blocks_[i].index), blocks_[i].rows,
                            blocks_[i].cols);
  }
  ElementView at(int i) {
    return ElementView(&data_.at(blocks_[i].index), blocks_[i].rows,
                       blocks_[i].cols);
  }
};

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

  void Clear() { data_.clear(); }

  /* Resizes storage to store `num_elements`. */
  void Resize(int num_elements) { data_.resize(num_elements); }

  /* Reserves storage to store `num_elements`. Size is not modified. */
  void Reserve(int num_elements) { data_.reserve(num_elements); }

  // Capcity to store Eigen elements.
  int elements_capacity() const { return data_.capacity(); }

  // Capacity to store scalar entries across all elements.
  int scalars_capacity() const {
    return data_.capacity() * EigenType::SizeAtCompileTime;
  }

  // Adds new element and returns mutable view to it.
  ElementView Add(int rows, int cols) {
    DRAKE_ASSERT(rows == EigenType::RowsAtCompileTime);
    DRAKE_ASSERT(cols == EigenType::ColsAtCompileTime);
    const int index = size();
    data_.emplace_back();
    return at(index);
  }

  // Adds new element and copies `data` into it.
  // @returns index to the new element.
  ElementView AddAndCopy(const EigenType& data) {
    return Add(data.rows(), data.cols()) = data;
  }

  void PushBack(const EigenType& data) { data_.push_back(data); }

  void SetZero() {
    for (auto& e : data_) {
      e.setZero();
    }
  }

  int size() const { return data_.size(); }
  ConstElementView at(int i) const { return data_.at(i); }
  ElementView at(int i) { return data_.at(i); }
};

// Choose fixed-size storage.
template <typename EigenType, int size_at_compile_time>
struct StorageSelector {
  using Storage = FixedSizeStorage<EigenType>;
};

// Choose dynamic-size storage.
template <typename EigenType>
struct StorageSelector<EigenType, Eigen::Dynamic> {
  using Storage = DynamicSizeStorage<EigenType>;
};

/* A replacement for std::vector<MatrixX<T>> that offers a contiguous layout of
 memory.

 @tparam EigenType The type of the Eigen elements. E.g. MatrixXd,
 Vector3, etc.
 @pre EigenType must be an Eigen type derived from Eigen::MatrixBase. */
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

  /* Default constructor for an empty pool. */
  EigenPool() = default;

  void Reserve(int num_elements)
    requires is_fixed_size_v<EigenType>
  {  // NOLINT(whitespace/braces)
    storage_.Reserve(num_elements);
  }

  /* Resizes the pool to store num_elements.
   Only available for fixed size Eigen tpes. */
  void Resize(int num_elements)
    requires is_fixed_size_v<EigenType>
  {  // NOLINT(whitespace/braces)
    storage_.Resize(num_elements);
  }

  /* Resize for a pool of VectorX elements of the given `sizes`. */
  void Resize(const std::vector<int>& sizes)
    requires is_dynamic_size_vector_v<EigenType>
  {  // NOLINT(whitespace/braces)
    storage_.Resize(sizes);
  }

  /* Resize for a pool of matrices with the specified `rows` and `cols`.
   rows (cols) are ignored if the rows (cols) of EigenType are fixed at compile
   time. */
  void Resize(const std::vector<int>& rows, const std::vector<int>& cols)
    requires(!is_fixed_size_v<EigenType>)
  {  // NOLINT(whitespace/braces)
    storage_.Resize(rows, cols);
  }

  // Reserves memory for `num_elements` of at most `max_size` scalars each.
  // For fixed size EigenType, max_size is ignored.
  void Reserve(int num_elements, int max_size) {
    storage_.Reserve(num_elements, max_size);
  }

  /* Clears data. Capacity is not changed, and thus memory is not freed. */
  void Clear() { storage_.Clear(); }

  void PushBack(const EigenType& data) { storage_.PushBack(data); }

  /* Adds element of the specified size and returns mutable to it. */
  ElementView Add(int rows, int cols) { return storage_.Add(rows, cols); }

  /* Adds new element and copies `data` into it.
   @returns mutable view to the new element. */
  ElementView AddAndCopy(const EigenType& data) {
    return storage_.AddAndCopy(data);
  }

  /* Sugar to add a data set into the pool. */
  // TODO(amcastro-tri): Consider more efficient, all-at-once, allocation.
  void PushBack(const std::vector<EigenType>& data) {
    for (const auto& d : data) {
      AddAndCopy(d);
    }
  }

  /* Zeroes out all elements in the pool. */
  void SetZero() { storage_.SetZero(); }

  /* Returns the number of elements in the pool. */
  int size() const { return storage_.size(); }

  /* Returns the maximum number of Eigen objects that can be stored without
  additional dynamics memory allocation. */
  int elements_capacity() const { return storage_.elements_capacity(); }

  /* Returns the capacity to store scalars. */
  int scalars_capacity() { return storage_.scalars_capacity(); }

  /* Const access to the i-th element. */
  const ConstElementView operator[](int i) const {
    DRAKE_ASSERT(0 <= i && i < size());
    return storage_.at(i);
  }

  /* Non-const access to the i-th element. */
  ElementView operator[](int i) {
    DRAKE_ASSERT(0 <= i && i < size());
    return storage_.at(i);
  }

  const std::vector<Scalar>& data() const { return storage_.data_; }

 private:
  Storage storage_;
};

}  // namespace pooled_sap
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
