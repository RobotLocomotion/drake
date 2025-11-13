#pragma once

#include <span>
#include <type_traits>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/ssize.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace icf {
namespace internal {

/* Contiguous storage for a pool of fixed-size Eigen objects.

This is essentially a wrapper around std::vector<EigenType>, where EigenType is
something like Matrix3d, Vector4d, etc.

All of the public types and methods are an implementation of the EigenPool API.
Refer to that class for documentation. */
template <typename EigenType>
class EigenPoolFixedSizeStorage {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(EigenPoolFixedSizeStorage);

  static_assert(EigenType::SizeAtCompileTime != Eigen::Dynamic);

  using Scalar = typename EigenType::Scalar;
  using ConstElementView = const EigenType&;
  using ElementView = EigenType&;

  EigenPoolFixedSizeStorage();
  ~EigenPoolFixedSizeStorage();

  int size() const { return ssize(data_); }
  ConstElementView operator[](int i) const { return data_[i]; }
  ElementView operator[](int i) { return data_[i]; }
  void Resize(int num_elements, int rows, int cols);
  void Clear();
  void SetZero();

 private:
  // Contiguous storage for all Eigen matrices.
  std::vector<EigenType> data_;
};

/* Contiguous storage for a pool of dynamic-size Eigen objects.

Each element in the pool can have a different size, but they must have the
same type (e.g. MatrixXd, VectorXd, VectorX<AutoDiffXd>, etc).

All of the public types and methods are an implementation of the EigenPool API.
Refer to that class for documentation. */
template <typename EigenType>
class EigenPoolDynamicSizeStorage {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(EigenPoolDynamicSizeStorage);

  static_assert(EigenType::SizeAtCompileTime == Eigen::Dynamic);

  using Scalar = typename EigenType::Scalar;
  using ConstElementView = Eigen::Map<const EigenType>;
  using ElementView = Eigen::Map<EigenType>;

  EigenPoolDynamicSizeStorage();
  ~EigenPoolDynamicSizeStorage();

  int size() const { return blocks_.size(); }
  ConstElementView operator[](int i) const {
    return ConstElementView(&data_[blocks_[i].index], blocks_[i].rows,
                            blocks_[i].cols);
  }
  ElementView operator[](int i) {
    return ElementView(&data_[blocks_[i].index], blocks_[i].rows,
                       blocks_[i].cols);
  }
  void Resize(int num_elements, int rows, int cols);
  /* Slight API change vs the outer class: if either RowsAtCompileTime or
  ColsAtCompileTime is not Eigen::Dynamic, then the respective `rows` or `cols`
  argument must be empty (vs a matching `int` in the outer class). */
  void Resize(int num_elements, std::span<const int> rows,
              std::span<const int> cols);
  void SetZero();
  void Clear();

 private:
  struct ElementData {
    int index{0};  // Offset into data_.
    int rows{0};   // Number of rows.
    int cols{0};   // Number of columns.
  };

  /* Appends a new element of the specified size to the end of the pool. */
  void Add(int rows, int cols);

  // Contiguous storage for all Eigen scalars.
  std::vector<Scalar> data_;

  // Properly sized maps to each element.
  std::vector<ElementData> blocks_;
};

/* A replacement for std::vector<Eigen::Matrix<...>> with a contiguous memory
layout.

@tparam EigenType The type of Eigen::Matrix being stored; valid choices are:
- Matrix3<T>
- Matrix3X<T>
- Matrix6<T>
- Matrix6X<T>
- MatrixX<T>
- Vector3<T>
- Vector6<T>
- VectorX<T>
where T is either `double` or `AutoDiffXd`. */
template <typename EigenType>
class EigenPool {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(EigenPool);

  static_assert(is_eigen_type<EigenType>::value);

  /* The underlying scalar type of the Eigen::Matrix in this pool. */
  using Scalar = typename EigenType::Scalar;

  /* The private implementation class. */
  using Storage =
      std::conditional_t<EigenType::SizeAtCompileTime == Eigen::Dynamic,
                         EigenPoolDynamicSizeStorage<EigenType>,
                         EigenPoolFixedSizeStorage<EigenType>>;

  /* The return type of const operator[]. For fixed-size storage, this is a C++
  reference to the EigenType. For dynamic-size stoage, this is an Eigen::Map. */
  using ConstElementView = typename Storage::ConstElementView;

  /* The return type of operator[]. For fixed-size storage, this is a C++
  reference to the EigenType. For dynamic-size stoage, this is an Eigen::Map. */
  using ElementView = typename Storage::ElementView;

  /* Constructs an empty pool. */
  EigenPool();

  ~EigenPool();

  /* Returns the number of elements in this pool. */
  int size() const { return ssize(storage_); }

  /* Returns a const reference (or Eigen::Map) to the i-th element. */
  const ConstElementView operator[](int i) const {
    DRAKE_DEMAND(0 <= i && i < size());
    return storage_[i];
  }

  /* Returns a mutable reference (or Eigen::Map) to the i-th element. */
  ElementView operator[](int i) {
    DRAKE_DEMAND(0 <= i && i < size());
    return storage_[i];
  }

  /* @name Re-sizing

  Resize methods allocate memory only if the current capacity is insufficient,
  and do not reduce the capacity.

  Note that even though Resize() is not a cheap / inline operation, these outer
  class wrappers are still defined inline so that the DRAKE_DEMAND checks can be
  elided by the compiler. The Storage methods that these delegate to a properly
  non-inline, so having these forwarding functions defined inline is not a big
  problem.

  @{ */

  /* Resizes a pool using a homogeneous size for all elements. If either or both
  of RowsAtCompileTime or ColsAtCompileTime are not Eigen::Dynamic, then the
  respective `rows` or `cols` argument must match the compile-time size. */
  void Resize(int num_elements, int rows, int cols) {
    DRAKE_DEMAND(EigenType::RowsAtCompileTime == Eigen::Dynamic ||
                 rows == EigenType::RowsAtCompileTime);
    DRAKE_DEMAND(EigenType::ColsAtCompileTime == Eigen::Dynamic ||
                 cols == EigenType::ColsAtCompileTime);
    storage_.Resize(num_elements, rows, cols);
  }

  /* Resizes a pool using heterogenous row sizes for the elements but a fixed
  size for `cols` that must match ColsAtCompileTime. */
  void Resize(int num_elements, std::span<const int> rows, int cols = 1)
    requires(EigenType::RowsAtCompileTime == Eigen::Dynamic &&
             EigenType::ColsAtCompileTime >= 0)
  {
    DRAKE_DEMAND(cols == EigenType::ColsAtCompileTime);
    storage_.Resize(num_elements, rows, {});
  }

  /* Resizes a pool using heterogenous column sizes for the elements but a fixed
  size for `rows` that must match RowsAtCompileTime. */
  void Resize(int num_elements, int rows, std::span<const int> cols)
    requires(EigenType::RowsAtCompileTime >= 0 &&
             EigenType::ColsAtCompileTime == Eigen::Dynamic)
  {
    DRAKE_DEMAND(rows == EigenType::RowsAtCompileTime);
    storage_.Resize(num_elements, {}, cols);
  }

  /* Resizes a pool using heterogenous sizes for the elements. Both
  RowsAtCompileTime and ColsAtCompileTime must be Eigen::Dynamic. */
  void Resize(int num_elements, std::span<const int> rows,
              std::span<const int> cols)
    requires(EigenType::RowsAtCompileTime == Eigen::Dynamic &&
             EigenType::ColsAtCompileTime == Eigen::Dynamic)
  {
    storage_.Resize(num_elements, rows, cols);
  }

  /* @} */

  /* Sets this pool's size() to zero. Capacity is not changed, and thus memory
  is not freed. */
  void Clear() { storage_.Clear(); }

  /* Zeroes out all elements in this pool. */
  void SetZero() { storage_.SetZero(); }

 private:
  /* The underlying data, one of EigenPool{Fixed,Dynamic}SizeStorage. */
  Storage storage_;
};

}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
