#include "drake/examples/BouncingBall/bouncing_ball-inl.h"

#include "drake/common/eigen_autodiff_types.h"

namespace drake {
namespace bouncingball {

template class DRAKESYSTEMFRAMEWORK_EXPORT BouncingBall<double>;
template class DRAKESYSTEMFRAMEWORK_EXPORT BouncingBall<AutoDiffXd>;

}  // namespace bouncingball
}  // namespace drake
