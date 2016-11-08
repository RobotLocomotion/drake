// NOLINTNEXTLINE(build/include) False positive on inl file.
#include "drake/examples/bouncing_ball/ball-inl.h"

#include "drake/common/eigen_autodiff_types.h"

namespace drake {
namespace bouncing_ball {

template class Ball<double>;
template class Ball<AutoDiffXd>;

}  // namespace bouncing_ball
}  // namespace drake
