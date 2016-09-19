#include "drake/examples/BouncingBall/ball-inl.h"

#include "drake/common/eigen_autodiff_types.h"

namespace drake {
namespace bouncingball {

template class DRAKESYSTEMFRAMEWORK_EXPORT Ball<double>;
template class DRAKESYSTEMFRAMEWORK_EXPORT Ball<AutoDiffXd>;

}  // namespace bouncingball
}  // namespace drake
