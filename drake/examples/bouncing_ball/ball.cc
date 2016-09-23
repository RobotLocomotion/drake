#include "drake/examples/bouncing_ball/ball-inl.h"

#include "drake/common/eigen_autodiff_types.h"
#include "drake/drakeBouncingBall_export.h"

namespace drake {
namespace bouncingball {

template class DRAKEBOUNCINGBALL_EXPORT Ball<double>;
template class DRAKEBOUNCINGBALL_EXPORT Ball<AutoDiffXd>;

}  // namespace bouncingball
}  // namespace drake
