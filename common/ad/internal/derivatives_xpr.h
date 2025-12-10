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

}  // namespace ad
}  // namespace drake
