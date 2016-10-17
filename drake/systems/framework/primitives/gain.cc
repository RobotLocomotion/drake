#include "drake/systems/framework/primitives/gain-inl.h"

#include "drake/common/eigen_autodiff_types.h"

namespace drake {
namespace systems {

template class DRAKE_EXPORT Gain<double>;
template class DRAKE_EXPORT Gain<AutoDiffXd>;
template class DRAKE_EXPORT Gain<Eigen::AutoDiffScalar<Eigen::Matrix<double, 2,
    1, Eigen::DontAlign>>>;

}  // namespace systems
}  // namespace drake
