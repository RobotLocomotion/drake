/* clang-format off */
// NOLINTNEXTLINE(build/include) False positive on inl file.
#include "drake/examples/bouncing_ball/bouncing_ball-inl.h"
/* clang-format on */

#include "drake/common/eigen_autodiff_types.h"

namespace drake {
namespace bouncing_ball {

template class BouncingBall<double>;
template class BouncingBall<AutoDiffXd>;

}  // namespace bouncing_ball
}  // namespace drake
