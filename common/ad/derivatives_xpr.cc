#include "drake/common/ad/derivatives_xpr.h"

#include "drake/common/drake_assert.h"

namespace drake {
namespace ad {

using Stride = internal::DerivativesStride;

DerivativesConstXpr::DerivativesConstXpr(double coeff, const double* data,
                                         int size, int stride)
    : internal::DerivativesConstXprBase(
          coeff * Eigen::Map<const Eigen::VectorXd, 0 /* Options */, Stride>(
                      data, size, Stride(0, stride))) {
  DRAKE_DEMAND(size == 0 || data != nullptr);
}

}  // namespace ad
}  // namespace drake
