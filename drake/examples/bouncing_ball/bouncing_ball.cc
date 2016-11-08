// NOLINTNEXTLINE(build/include) False positive on inl file.
#include "drake/examples/bouncing_ball/bouncing_ball-inl.h"

#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/drake_export.h"

namespace drake {
namespace bouncing_ball {

template class BouncingBall<double>;
template class BouncingBall<AutoDiffXd>;

}  // namespace bouncing_ball
}  // namespace drake
