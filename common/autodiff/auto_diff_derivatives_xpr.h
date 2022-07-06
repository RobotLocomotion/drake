#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace internal {

// This forward-declaration is defined by common/autodiff/interal/partials.h.
class Partials;

/* The stride class for autodiff::AutoDiffDerivativesConstXpr. */
using AutoDiffDerivativesStride = Eigen::Stride<0, Eigen::Dynamic>;

/* The base class of autodiff::AutoDiffDerivativesConstXpr. */
using AutoDiffDerivativesConstXprBase =
    decltype(0.0 * Eigen::Map<
        const Eigen::VectorXd, 0 /* Options */, AutoDiffDerivativesStride>{
            static_cast<const double*>(nullptr),
            Eigen::Index{0},
            AutoDiffDerivativesStride{0, 0}});

}  // namespace internal

namespace autodiff {

/** The return type for AutoDiff::derivatives() when the AutoDiff is const.

The underlying type is similar to an Eigen::Map<const Eigen::VectorXd> which
includes all of the usual functions for element access and arithmetic.

Additionally, we provide a with_explicit_zeros() function to easily inflate
an empty vector into a vector of zeros. This is especially useful for unit
tests that need to inspect gradients.

(Advanced) To be more specific, since we need to scale up the Map by a
constant, the underlying type actually goes something like this:
  CwiseBinaryOp<
    scalar_product_op,
    const CwiseNullaryOp<scalar_constant_op>(coeff),
    const Map(data, size)>> */
class AutoDiffDerivativesConstXpr final
    : public internal::AutoDiffDerivativesConstXprBase {
 public:
  /** This class is copyable, but not copy-assignable. */
  AutoDiffDerivativesConstXpr(const AutoDiffDerivativesConstXpr&) = default;
  void operator=(const AutoDiffDerivativesConstXpr&) = delete;

  /** Convenience function for promoting empty derivatives to a zero vector.

  If this->size() is zero, then returns a new Xpr with result.size() == size
  that points to a vector of all zeros (ala VectorXd::Zero(size)).

  If this->size() is already equal to size, then simply returns an unmodified
  copy of this.

  @note This does *not* alter the dervatives of the AutoDiff object that
  returned this proxy object. (This is a const Xpr.)

  @throws std::exception If this->size() was already non-zero and is not equal
  to the requested size. */
  AutoDiffDerivativesConstXpr with_explicit_zeros(Eigen::Index size);

  /** This class does not support resize(). */
  template <typename... Args>
  void resize(Args...) const = delete;

  /** This class does not support conservativeResize(). */
  template <typename... Args>
  void conservativeResize(Args...) const = delete;

 private:
  // We can only be constructed by class Partials.
  friend internal::Partials;
  explicit AutoDiffDerivativesConstXpr(
      double coeff, const double* data, int size, int stride);
};

/** The return type for AutoDiff::derivatives() when the AutoDiff is mutable.

The underlying type is Eigen::Map<Eigen::VectorXd> which includes all of the
usual functions for element access and arithmetic.

Additionally, we provide a with_explicit_zeros() function to easily inflate
an empty vector into a vector of zeros. This is especially useful for unit
tests that need to set or inspect gradients.

@note Unlike a typical Map, this class also provides the capability to resize()
the underlying data as well as assign through from a vector of different size.

@warning After resizing or assigning to this object, `this` is no longer valid.
Always use those functions' return value as the ongoing means of access. */
class AutoDiffDerivativesMutableXpr final : public Eigen::Map<Eigen::VectorXd> {
 public:
  /** This class is copyable, but not copy-assignable. */
  AutoDiffDerivativesMutableXpr(const AutoDiffDerivativesMutableXpr&) = default;
  void operator=(const AutoDiffDerivativesMutableXpr&) = delete;

  /** Sets the AutoDiff::derivatives() to a new value and returns a new Xpr.
  @warning After calling this object, `this` is no longer valid. Always use the
  return value as the ongoing means of access, or call AutoDiff::derivatives()
  again to obtain a new reference. */
  template <typename Derived>
  AutoDiffDerivativesMutableXpr operator=(const DenseBase<Derived>& other) {
    return SetFrom(other);
  }

  /** Convenience function for promoting empty derivatives to a zero vector.

  If this->size() is zero, then returns a new Xpr with result.size() == size
  that points to a vector of all zeros (ala VectorXd::Zero(size)).

  If this->size() is already equal to size, then simply returns an unmodified
  copy of this.

  @note This *does* alter the dervatives of the AutoDiff object that returned
  this proxy object.

  @throws std::exception If this->size() was already non-zero and is not equal
  to the requested size. */
  AutoDiffDerivativesMutableXpr with_explicit_zeros(Eigen::Index size);

  /** Like MatrixBase::resize(), discards any existing data and resizes the
  derivatives vector.
  @warning After calling this object, `this` is no longer valid. Always use the
  return value as the ongoing means of access.
  @pre rows >= 0
  @throws std::exception when cols != 1 */
  AutoDiffDerivativesMutableXpr resize(Eigen::Index rows,
                                       Eigen::Index cols = 1);

  /** Like MatrixBase::conservativeResize(), changes the size() of this while
  keeping existing data intact. New values are filled with zeros.
  @warning After calling this object, `this` is no longer valid. Always use the
  return value as the ongoing means of access.
  @pre rows >= 0
  @throws std::exception when cols != 1 */
  AutoDiffDerivativesMutableXpr conservativeResize(Eigen::Index rows,
                                                   Eigen::Index cols = 1);

 private:
  // We can only be constructed by class Partials.
  friend internal::Partials;
  explicit AutoDiffDerivativesMutableXpr(internal::Partials* backrefrence,
      double* data, int size);

  // Implementation of operator=.
  AutoDiffDerivativesMutableXpr SetFrom(
      const Eigen::Ref<const Eigen::VectorXd>& other);

  internal::Partials* backreference_{};
};

}  // namespace autodiff
}  // namespace drake
