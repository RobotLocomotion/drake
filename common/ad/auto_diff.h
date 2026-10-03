#pragma once

#include "drake/common/ad/internal/derivatives_xpr.h"
#include "drake/common/ad/internal/partials.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/fmt.h"

namespace drake {
namespace ad {

/** A scalar type that performs automatic differentiation, similar to
`Eigen::AutoDiffScalar<Eigen::VectorXd>`. Unlike `Eigen::AutoDiffScalar`,
Drake's AutoDiff is not templated; it only supports dynamically-sized
derivatives using floating-point doubles.

However, by using careful representation tricks (maintaining a separate scale
factor, and using inline storage in case only one partial is non-zero) it runs
faster than `Eigen::AutoDiffScalar<Eigen::VectorXd>`. */
class AutoDiff {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(AutoDiff);

  /** Compatibility alias to mimic Eigen::AutoDiffScalar. */
  using DerType = Eigen::VectorXd;

  /** Compatibility alias to mimic Eigen::AutoDiffScalar. */
  using Scalar = double;

  /** Constructs zero. */
  AutoDiff() = default;

  /** Constructs a value with empty derivatives. */
  // NOLINTNEXTLINE(runtime/explicit): This conversion is desirable.
  AutoDiff(double value) : value_{value} {}

  /** Constructs a value with a single partial derivative of 1.0 at the given
  `offset` in a vector of `size` otherwise-zero derivatives. */
  AutoDiff(double value, Eigen::Index size, Eigen::Index offset)
      : value_{value}, partials_{size, offset} {}

  /** Constructs a value with the given derivatives. */
  AutoDiff(double value, const Eigen::Ref<const Eigen::VectorXd>& derivatives)
      : value_{value}, partials_{derivatives} {}

  /** Assigns a value and clears the derivatives. */
  AutoDiff& operator=(double value) {
    value_ = value;
    partials_.SetZero();
    return *this;
  }

  ~AutoDiff() = default;

  /** Returns the value part of this %AutoDiff (readonly). */
  double value() const { return value_; }

  /** (Advanced) Returns the value part of this %AutoDiff (mutable).
  Operations on this value will NOT alter the derivatives. */
  double& value() { return value_; }

  /** Returns a view of the derivatives part of this %AutoDiff (readonly). */
  DerivativesConstXpr derivatives() const { return partials_.make_const_xpr(); }

  /** (Advanced) Returns a mutable view of the derivatives part of this
  %AutoDiff.

  Instead of mutating the derivatives after construction, it's generally
  preferable to set them directly in the constructor if possible.

  @note This function name is kept for compatibility with Eigen::AutoDiffScalar
  but it does NOT run in constant-time even though its name is lowercase.
  Calling this function often needs to finalize the derivatives prior to
  returning the reference, so is O(N) in the size of the derivatives. */
  DerivativesMutableXpr derivatives() { return partials_.MakeMutableXpr(); }

  /// @name Internal use only
  //@{

  /** (Internal use only)
  Users should call derivatives() instead. */
  const internal::Partials& partials() const { return partials_; }

  /** (Internal use only)
  Users should call derivatives() instead. */
  internal::Partials& partials() { return partials_; }

  //@}

 private:
  double value_{0.0};
  internal::Partials partials_;
};

}  // namespace ad
}  // namespace drake

// These further refine our AutoDiff type and must appear in exactly this order.
// clang-format off
#include "drake/common/ad/internal/standard_operations.h"
#include "drake/common/ad/internal/eigen_specializations.h"
#include "drake/common/ad/internal/drake_operations.h"
// clang-format on

/* Formats the `value()` part of x to the stream.
To format the derivatives use `drake::fmt_eigen(x.derivatives())`. */
DRAKE_FORMATTER_AS(, drake::ad, AutoDiff, x, x.value())
