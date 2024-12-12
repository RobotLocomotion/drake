#include "drake/multibody/mpm/bspline_weights.h"

#include "drake/math/autodiff_gradient.h"

namespace drake {
namespace multibody {
namespace mpm {
namespace internal {

BsplineWeights<double> MakeBsplineWeights(const Vector3<AutoDiffXd>& x,
                                          double dx) {
  const auto x_double = math::DiscardZeroGradient(x);
  return BsplineWeights<double>(x_double, dx);
}

}  // namespace internal
}  // namespace mpm
}  // namespace multibody
}  // namespace drake
