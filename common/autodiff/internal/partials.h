#pragma once

#include <atomic>
#include <cstdlib>
#include <utility>

#include "drake/common/autodiff/derivatives_xpr.h"
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
      if (data_ != nullptr) {
        DecrementUseCount();
      }
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

  /* Resets this back to null.
  @pre data() != nullptr */
  void ResetAssumingNonNull() {
    DRAKE_ASSERT(data_ != nullptr);
    DecrementUseCount();
    data_ = nullptr;
  }

  /* Returns the double array storage (or null, when empty). */
  const double* data() const { return data_; }
  double* mutable_data() { return data_; }

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

  /* Constructs a single partial derivative of `coeff` at the given `offset` in
  a vector of `size` otherwise-zero derivatives.
  @throws std::exception if offset >= size */
  Partials(Eigen::Index size, Eigen::Index offset, double coeff = 1.0);

  /* Constructs a vector with a copy of the given value. */
  explicit Partials(const Eigen::Ref<const Eigen::VectorXd>& value);

  /* Constructs a vector with a copy of the given value.
  @pre value.size() >= 2 */
  explicit Partials(const autodiff::DerivativesConstXpr& value);

  ~Partials() = default;

  /* Returns the size of this vector. */
  int size() const {
    // This is a careful representational choice so that calculating the
    // effective size() from magic_size_ is just two branchless instructions.
    // In particular, it compiles down to `(magic_size_ >> 31) ^ magic_size_`.
    // The other obvious alternative would be to use `std::abs(magic_size_)`
    // but that adds one extra instruction to tweak the complement by 1.
    return (magic_size_ >= 0) ? int{magic_size_} : (-magic_size_ - 1);
  }

  /* Returns true when this denotes a value known to be zero vector -- either a
  zero-sized vector, or a vector filled with zeros. This might return false in
  the rare case of a dense vector addition that canceled out to zero. */
  bool is_known_zero() const { return magic_size_ <= 0; }

  /* Returns an Eigen-compatible view into this vector. */
  autodiff::DerivativesConstXpr make_const_xpr() const;

  /* Returns an Eigen-compatible mutable view into this vector, including
  resizing. This is expensive (makes a copy). */
  autodiff::DerivativesMutableXpr MakeMutableXpr();

  /* Set this to zero. */
  void SetZero() {
    if (magic_size_ <= 0) {
      return;
    }
    magic_size_ = -magic_size_ - 1;
    if (unit_ == 0) {
      // Freeing the storage immediately is important for performance so that
      // future copies of `this` will be cheap (no operations on use_count()).
      storage_.ResetAssumingNonNull();
    }
  }

  /* Scales this vector by the given amount. */
  void Mul(double factor) {
    if (factor == 0.0) {
      SetZero();
    } else {
      coeff_ *= factor;
    }
  }

  /* Scales this vector by the reciprocal of the given amount. */
  void Div(double factor) {
    coeff_ /= factor;
  }

  /* Adds `other` into `this`. */
  void Add(const Partials& other) {
    AddScaled(1.0, other);
  }

  /* Adds `scale * other` into `this`. */
  void AddScaled(double scale, const Partials& other) {
    // XXX This logic needs better comments.
    // XXX Should we be checking for matched sizes in Debug builds?
    if (other.magic_size_ <= 0) {
      if (magic_size_ == 0) {
        magic_size_ = other.magic_size_;
      }
      return;
    }
    if (scale == 0.0) {
      return;
    }
    if (is_known_zero()) {
      // Borrow from `other` with no new allocations.
      *this = other;
      // N.B. Only _after_ copying can we apply the scale.
      coeff_ *= scale;
      return;
    }
    AddScaledImpl(scale, other);
  }

 private:
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
  friend autodiff::DerivativesMutableXpr;
  autodiff::DerivativesMutableXpr SetFrom(
      const Eigen::Ref<const Eigen::VectorXd>& other);

  void CheckInvariants() const;

  // Refer to CheckInvariants() commentary in the cc file for what these denote.
  reset_after_move<int32_t> magic_size_{0};
  int32_t unit_{0};
  double coeff_{0.0};
  CowVec storage_;
};

}  // namespace internal
}  // namespace drake
