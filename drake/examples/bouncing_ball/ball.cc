// NOLINTNEXTLINE(build/include) False positive on inl file.
#include "drake/examples/bouncing_ball/ball-inl.h"

#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/drake_export.h"

namespace drake {
namespace bouncing_ball {

template class DRAKE_EXPORT Ball<double>;
template class DRAKE_EXPORT Ball<AutoDiffXd>;

}  // namespace bouncing_ball
}  // namespace drake
