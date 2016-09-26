#include "drake/examples/bouncing_ball/ball-inl.h"

#include "drake/common/eigen_autodiff_types.h"
#include "drake/drakeBouncingBall_export.h"

namespace drake {
namespace bouncing_ball {

template class DRAKEBOUNCINGBALL_EXPORT Ball<double>;
template class DRAKEBOUNCINGBALL_EXPORT Ball<AutoDiffXd>;

}  // namespace bouncing_ball
}  // namespace drake
