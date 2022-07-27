#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace ad {
namespace internal {

// This forward-declaration is defined by common/autodiff/interal/partials.h.
class Partials;

/* The stride class for ad::DerivativesConstXpr. */
using DerivativesStride = Eigen::Stride<0, Eigen::Dynamic>;

/* The base class of ad::DerivativesConstXpr. */
using DerivativesConstXprBase =
    decltype(0.0 * Eigen::Map<
        const Eigen::VectorXd, 0 /* Options */, DerivativesStride>{
            static_cast<const double*>(nullptr),
            Eigen::Index{0},
            DerivativesStride{0, 0}});

}  // namespace internal

/** The return type for AutoDiff::derivatives() when the AutoDiff is const.

The underlying type is similar to an `Eigen::Map<const Eigen::VectorXd>` which
includes all of the usual functions for element access and arithmetic.

Additionally, we provide a with_explicit_zeros() function to easily inflate
an empty vector into a vector of zeros. This is especially useful for unit
tests that need to inspect gradients.

(Advanced) To be more specific, because we need to scale up the Map by a
constant, the underlying type actually goes something like this:
<tt>
Eigen::CwiseBinaryOp<
    Eigen::scalar_product_op,
    const Eigen::CwiseNullaryOp<Eigen::scalar_constant_op>(coeff),
    const Eigen::Map(data, size)>>
</tt> */
class DerivativesConstXpr final
    : public internal::DerivativesConstXprBase {
 public:
  /** This class is copyable, but not copy-assignable. */
  DerivativesConstXpr(const DerivativesConstXpr&) = default;
  void operator=(const DerivativesConstXpr&) = delete;

  /** Convenience function for promoting empty derivatives to a zero vector.

  If this->size() is already equal to size, then simply returns an unmodified
  copy of this.

  If this->size() is zero, then returns a new Xpr with `result.size() == size`
  that points to a vector of all zeros (ala VectorXd::Zero(size)).

  @note This does *not* alter the dervatives of the AutoDiff object that
  returned this proxy object. (This is a const Xpr.)

  @throws std::exception If this->size() was already non-zero and is not equal
  to the requested size. */
  DerivativesConstXpr with_explicit_zeros(Eigen::Index size);

  /** This class does not support resize(). */
  void resize(Eigen::Index, Eigen::Index = {}) const = delete;

  /** This class does not support conservativeResize(). */
  void conservativeResize(Eigen::Index, Eigen::Index = {}) const = delete;

 private:
  // We can only be constructed by class Partials.
  friend internal::Partials;
  explicit DerivativesConstXpr(
      double coeff, const double* data, int size, int stride);
};

/** The return type for AutoDiff::derivatives() when the AutoDiff is mutable.

The underlying type is `Eigen::Map<Eigen::VectorXd>` which includes all of the
usual functions for element access and arithmetic.

Additionally, we provide a with_explicit_zeros() function to easily inflate
an empty vector into a vector of zeros. This is especially useful for unit
tests that need to set or inspect gradients. Note that this function *does*
mutate the underlying AutoDiff object derivatives.

@note Unlike a typical Map, this class also provides the capability to resize()
the underlying data as well as assign through from a vector of different size.

@warning After resizing or assigning to this object, `this` is no longer valid.
Always use those functions' return value as the ongoing means of access. */
class DerivativesMutableXpr final : public Eigen::Map<Eigen::VectorXd> {
 public:
  /** This class is copyable, but not copy-assignable. */
  DerivativesMutableXpr(const DerivativesMutableXpr&) = default;
  void operator=(const DerivativesMutableXpr&) = delete;

  /** Sets the AutoDiff::derivatives() to a new value and returns a new Xpr.
  @warning After calling this object, `this` is no longer valid. Always use the
  return value as the ongoing means of access, or call AutoDiff::derivatives()
  again to obtain a new reference. */
  template <typename Derived>
  DerivativesMutableXpr operator=(const DenseBase<Derived>& other) {
    return SetFrom(other);
  }

  /** Convenience function for promoting empty derivatives to a zero vector.

  If this->size() is already equal to size, then simply returns an unmodified
  copy of this.

  If this->size() is zero, then returns a new Xpr with result.size() == size
  that points to a vector of all zeros (ala VectorXd::Zero(size)).

  @note This *does* alter the dervatives of the AutoDiff object that returned
  this proxy object.

  @throws std::exception If this->size() was already non-zero and is not equal
  to the requested size. */
  DerivativesMutableXpr with_explicit_zeros(Eigen::Index size);

  /** Like MatrixBase::resize(), discards any existing data and resizes the
  derivatives vector.
  @warning After calling this object, `this` is no longer valid. Always use the
  return value as the ongoing means of access.
  @pre rows >= 0
  @throws std::exception when cols != 1 */
  DerivativesMutableXpr resize(Eigen::Index rows, Eigen::Index cols = 1);

  /** Like MatrixBase::conservativeResize(), changes the size() of this while
  keeping existing data intact. New values are filled with zeros.
  @warning After calling this object, `this` is no longer valid. Always use the
  return value as the ongoing means of access.
  @pre rows >= 0
  @throws std::exception when cols != 1 */
  DerivativesMutableXpr conservativeResize(Eigen::Index rows,
                                           Eigen::Index cols = 1);

 private:
  // We can only be constructed by class Partials.
  friend internal::Partials;
  explicit DerivativesMutableXpr(internal::Partials* backrefrence,
      double* data, int size);

  // Implementation of operator=.
  DerivativesMutableXpr SetFrom(
      const Eigen::Ref<const Eigen::VectorXd>& other);

  internal::Partials* backreference_{};
};

}  // namespace ad
}  // namespace drake
