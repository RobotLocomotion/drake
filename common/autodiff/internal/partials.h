#pragma once

#include <atomic>
#include <cstdlib>
#include <utility>

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
    std::swap(data_, other.data_);
  }

  /* Swaps our storage with `other`. */
  CowVec& operator=(CowVec&& other) noexcept {
    std::swap(data_, other.data_);
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
      UseCount& counter = mutable_use_count();
      if (--counter == 0) {
        counter.~UseCount();
        std::free(&counter);
      }
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

  double* data_{nullptr};
};

class PartialsXpr;

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

  /* Returns an Eigen-compatible view into this vector. The return type here
  goes something like this:
    CwiseBinaryOp<
      scalar_product_op,
      const CwiseNullaryOp<scalar_constant_op>(coeff),
      const Map(data, size)>> */
  auto view_as_eigen() const {
    using StorageMap = Eigen::Map<const Eigen::VectorXd>;
    return coeff_ * StorageMap{view_as_eigen_data(), size_};
  }

  /* Returns an Eigen-compatible mutable view into this vector, including
  resizing. This is expensive (makes a copy). */
  PartialsXpr MakeMutableXpr();

  /* Set this to be an empty vector. */
  void Clear() {
    *this = {};
  }

  /* Scales this vector by the given amount. */
  void Mul(double scale) {
    if (scale == 0.0) {
      size_ = 0;
      unit_ = 0;
      coeff_ = 0.0;
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
  void AddImpl(const Partials&);
  void AddScaledImpl(double, const Partials&);

  bool is_unit() const { return unit_ > 0; }
  int get_unit() const {
    DRAKE_ASSERT(unit_ > 0);
    return unit_ - 1;
  }

  const double* view_as_eigen_data() const;

  friend PartialsXpr;
  // For use by PartialsXpr.
  PartialsXpr SetFrom(const Eigen::Ref<const Eigen::VectorXd>& other);

  void CheckInvariants() const;
  // Representation invariant:
  // - size_ >= 0
  // - if size_ > 0 then ∀i:
  //   - coeff != 0
  //   - if storage.data is non-null then:
  //     - storage.data must contain storage for exactly size_ elements
  // Abstraction function:
  // - When size == 0, the vector's value is zero (aka empty).
  // - When size > 0, the vector's value is coeff * storage.data,
  //   where null data also denotes a zero vector.
  // XXX ^^ this is wrong now
  reset_after_move<int32_t> size_{0};
  reset_after_move<int32_t> unit_{0};
  reset_after_move<double> coeff_{0.0};
  CowVec storage_;
};

/* The return type for Partials::MakeMutableXpr. This quacks like a mutable
VectorXd reference, including assignment that allows for resizing. */
class PartialsXpr final : public Eigen::Map<Eigen::VectorXd> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(PartialsXpr);
  explicit PartialsXpr(Partials* backrefrence);

  template <typename Derived>
  PartialsXpr operator=(const DenseBase<Derived>& other) {
    return backreference_->SetFrom(other);
  }

  void resize(Eigen::Index rows);
  void resize(Eigen::Index rows, Eigen::Index cols);

  // This proxy class does not (yet) support conservative resizing.
  template <typename... Args>
  void conservativeResize(Args... args) const = delete;

 private:
  Partials* backreference_{};
};

}  // namespace internal
}  // namespace drake
