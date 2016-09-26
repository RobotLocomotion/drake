#include "drake/examples/bouncing_ball/bouncing_ball-inl.h"

#include "drake/common/eigen_autodiff_types.h"
#include "drake/drakeBouncingBall_export.h"

namespace drake {
namespace bouncing_ball {

template class DRAKEBOUNCINGBALL_EXPORT BouncingBall<double>;
template class DRAKEBOUNCINGBALL_EXPORT BouncingBall<AutoDiffXd>;

}  // namespace bouncing_ball
}  // namespace drake
