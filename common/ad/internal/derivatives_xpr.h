#pragma once

#include "drake/common/eigen_types.h"

namespace drake {
namespace ad {
namespace internal {

// This forward-declaration is defined by drake/common/ad/interal/partials.h.
class Partials;

/* The stride class for ad::DerivativesConstXpr. (It uses a dynamic stride,
in anticipation of storage layout optimizations in the future.) */
using DerivativesStride = Eigen::Stride<
    /* OuterStrideAtCompileTime = */ 0,
    /* InnerStrideAtCompileTime = */ Eigen::Dynamic>;

/* The base class of ad::DerivativesConstXpr. It is the type of an Eigen
expression tree where a Map<VectorXd> is scaled elementwise by a constant. */
using DerivativesConstXprBase = decltype(  // BR
    0.0 *
    Eigen::Map<const Eigen::VectorXd, /* Options = */ 0, DerivativesStride>{
        static_cast<const double*>(nullptr), Eigen::Index{0},
        DerivativesStride{0, 0}});

}  // namespace internal

/** The return type for AutoDiff::derivatives() when the AutoDiff is const.

The underlying type is similar to an `Eigen::Map<const Eigen::VectorXd>` which
includes all of the usual functions for element access and arithmetic.

(Advanced) To be more specific, because we need to scale up the Map by a
constant, the underlying type actually goes something like this:
<tt>
Eigen::CwiseBinaryOp<
    Eigen::scalar_product_op,
    const Eigen::CwiseNullaryOp<Eigen::scalar_constant_op>(coeff),
    const Eigen::Map(data, size)>>
</tt> */
class DerivativesConstXpr final : public internal::DerivativesConstXprBase {
 public:
  /** This class is copyable, but not copy-assignable. */
  DerivativesConstXpr(const DerivativesConstXpr&) = default;
  void operator=(const DerivativesConstXpr&) = delete;

  /** This class does not support resize(). */
  void resize(Eigen::Index, Eigen::Index = {}) const = delete;

  /** This class does not support conservativeResize(). */
  void conservativeResize(Eigen::Index, Eigen::Index = {}) const = delete;

 private:
  // We can only be constructed by class Partials.
  friend internal::Partials;
  explicit DerivativesConstXpr(double coeff, const double* data, int size,
                               int stride);
};

/** The return type for AutoDiff::derivatives() when the AutoDiff is mutable.

The underlying type is `Eigen::Map<Eigen::VectorXd>` which includes all of the
usual functions for element access and arithmetic.

@note Unlike a typical Map, this class also provides the capability to resize()
the underlying data as well as assign through from a vector of different size.

@warning After resizing or assigning to this object, `this` is no longer valid.
Always use those functions' return value as the ongoing means of access. */
class DerivativesMutableXpr final : public Eigen::Map<Eigen::VectorXd> {
 public:
  /** This class is copyable and copy-assignable. */
  DerivativesMutableXpr(const DerivativesMutableXpr&) = default;
  DerivativesMutableXpr operator=(const DerivativesMutableXpr& other) {
    return SetFrom(other);
  }

  /** Sets the AutoDiff::derivatives() to a new value and returns a new Xpr.
  @warning After calling this object, `this` is no longer valid. Always use the
  return value as the ongoing means of access, or call AutoDiff::derivatives()
  again to obtain a new reference. */
  template <typename Derived>
  DerivativesMutableXpr operator=(const DenseBase<Derived>& other) {
    return SetFrom(other);
  }

  /** Like MatrixBase::resize(), discards any existing data and resizes the
  derivatives vector.
  @note The signature and defaults match MatrixBase::resize().
  @warning After calling this object, `this` is no longer valid. Always use the
  return value as the ongoing means of access.
  @pre rows >= 0
  @throws std::exception when cols != 1 */
  DerivativesMutableXpr resize(Eigen::Index rows, Eigen::Index cols = 1);

  /** Like MatrixBase::conservativeResize(), changes the size() of this while
  keeping existing data intact. New values are filled with zeros.
  @note The signature and defaults match MatrixBase::conservativeResize().
  @warning After calling this object, `this` is no longer valid. Always use the
  return value as the ongoing means of access.
  @pre rows >= 0
  @throws std::exception when cols != 1 */
  DerivativesMutableXpr conservativeResize(Eigen::Index rows,
                                           Eigen::Index cols = 1);

 private:
  // We can only be constructed by class Partials.
  friend internal::Partials;
  explicit DerivativesMutableXpr(internal::Partials* backreference,
                                 double* data, int size);

  // Implementation of operator=.
  DerivativesMutableXpr SetFrom(const Eigen::Ref<const Eigen::VectorXd>& other);

  internal::Partials* backreference_{};
};

}  // namespace ad
}  // namespace drake
