#pragma once

#include <atomic>
#include <cstdlib>
#include <utility>

#include "drake/common/autodiff/auto_diff_derivatives.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/reset_after_move.h"

namespace drake {
namespace internal {

/* Reference-counted heap storage for an array of doubles, for use by the
Partials class later in this file. The storage can be empty (null). */
class CowVec {
 public:
  /* Creates empty (null) storage. */
  CowVec() = default;

  /* Allocates new storage of the given size, but does not initialize it.
  If the size is zero, the storage will be empty (null). */
  static CowVec Allocate(int size);

  /* Copies the given value into newly-allocated storage.
  If the size is zero, the storage will be empty (null). */
  static CowVec Copy(const Eigen::Ref<const Eigen::VectorXd>& value);

  /* Steals the storage from `other`. */
  CowVec(CowVec&& other) noexcept {
    data_ = other.data_;
    other.data_ = nullptr;
  }

  /* Steals the storage from `other`. */
  CowVec& operator=(CowVec&& other) noexcept {
    if (this != &other) {
      if (data_ != nullptr) {
        DecrementUseCount();
      }
      data_ = other.data_;
      other.data_ = nullptr;
    }
    return *this;
  }

  /* Borrows the storage from `other` (incrementing use_count). */
  CowVec(const CowVec& other) noexcept {
    data_ = other.borrow_data();
  }

  /* Borrows the storage from `other` (incrementing use_count). */
  CowVec& operator=(const CowVec& other) noexcept {
    if (this != &other) {
      data_ = other.borrow_data();
    }
    return *this;
  }

  /* Decrements the storage use_count, freeing iff it reached zero. */
  ~CowVec() {
    if (data_ != nullptr) {
      DecrementUseCount();
    }
  }

  /* Returns the double array storage (or null, when empty). */
  const double* data() const { return data_; }

  /* Returns the current use count. Be careful when using this for control
  flow. Values other than 0 or 1 are subject to TOCTOU races. */
  int64_t get_use_count() const {
    return (data_ == nullptr) ? 0 : mutable_use_count().load();
  }

 private:
  using UseCount = std::atomic_int_fast64_t;

  /* Returns a mutable reference to our atomic use_count integer.
  @pre data_ is non-null (the use_count is part of the managed block). */
  UseCount& mutable_use_count() const {
    DRAKE_ASSERT(data_ != nullptr);
    return *reinterpret_cast<UseCount*>(
        reinterpret_cast<char*>(data_) - sizeof(UseCount));
  }

  /* For use by our copy constructor and copy assignment.
  Increments the use_count (when present) and returns the data pointer. */
  double* borrow_data() const {
    if (data_ != nullptr) {
      ++mutable_use_count();
    }
    return data_;
  }

  void DecrementUseCount();

  double* data_{nullptr};
};

/* A vector of partial derivatives, optimized for use with Drake's AutoDiff.

Partials are dynamically sized, and can have size() == 0.
When adding two non-zero Partials, both must be of the same size().

A large portion of this class definition appears inline, because AutoDiff is
used heavily in inner loops. The general approach is that simple word-sized
sets, gets, and branches appear inline; functions that do more than a couple
branches or touch more than a couple words (and all functions that allocate)
are out-of-line (in the cc file). */
class Partials {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Partials);

  /* Constructs an empty vector. */
  Partials() = default;

  // XXX add `coeff = 1.0` optional third argument.
  /* Constructs a single partial derivative of 1.0 at the given `offset` in a
  vector of `size` otherwise-zero derivatives. */
  Partials(Eigen::Index size, Eigen::Index offset);

  /* Constructs a vector with a copy of the given value. */
  explicit Partials(const Eigen::Ref<const Eigen::VectorXd>& value);

  ~Partials() = default;

  /* Returns the size of this vector. */
  int size() const { return size_; }

  /* Returns true iff size() == 0. */
  bool empty() const { return size_ == 0; }

  /* Returns an Eigen-compatible view into this vector. */
  autodiff::AutoDiffDerivativesConstXpr make_const_xpr() const;

  /* Returns an Eigen-compatible mutable view into this vector, including
  resizing. This is expensive (makes a copy). */
  autodiff::AutoDiffDerivativesMutableXpr MakeMutableXpr();

  /* Set this to be an empty vector. */
  void Clear() {
    size_ = 0;
    // Freeing the storage immediately is important for performance so that
    // future copies of `this` will be cheap (no operations on use_count()).
    storage_ = {};
  }

  /* Scales this vector by the given amount. */
  void Mul(double scale) {
    if (scale == 0.0) {
      Clear();
    } else {
      coeff_ *= scale;
    }
  }

  /* Adds `other` into `this`. */
  void Add(const Partials& other) {
    if (!other.empty()) {
      AddImpl(other);
    }
  }

  /* Adds `scale * other` into `this`. */
  void AddScaled(double scale, const Partials& other) {
    if (scale != 0.0 && !other.empty()) {
      AddScaledImpl(scale, other);
    }
  }

 private:
  /* The non-inline implementation of Add(). */
  void AddImpl(const Partials&);

  /* The non-inline implementation of AddScaled(). */
  void AddScaledImpl(double, const Partials&);

  /* Returns true iff this represents a (scaled) unit vector. */
  bool is_unit() const { return unit_ > 0; }

  /* Returns the index of the non-zero element of this (scaled) unit vector.
  @pre is_unit() */
  int get_unit_index() const {
    DRAKE_ASSERT(unit_ > 0);
    return unit_ - 1;
  }

  // Our MutableXpr type is allowed to set us via a backreference.
  friend autodiff::AutoDiffDerivativesMutableXpr;
  autodiff::AutoDiffDerivativesMutableXpr SetFrom(
      const Eigen::Ref<const Eigen::VectorXd>& other);

  void CheckInvariants() const;
  // Representation invariant:
  // - size >= 0
  // - unit >= 0
  // - if size == 0:
  //   - storage.data is null
  // - if size > 0:
  //   - coeff != 0.0
  //   - unit <= size
  //   - if unit == 0:
  //     - storage.data is non-null
  //     - storage.data contains storage for exactly size elements
  // Abstraction function:
  // - When size == 0, the partials value value is zero eveywhere.
  // - When size > 0 and unit > 0, the partials value is zero everywhere except
  //   for the (unit-1)'th partial where it has the value of coeff.
  // - When size > 0 and unit == 0 the partials value is coeff * storage where
  //   the storage is a full-sized dense vector on the heap.
  reset_after_move<int32_t> size_{0};
  int32_t unit_{0};
  double coeff_{0.0};
  CowVec storage_;
};

}  // namespace internal
}  // namespace drake
