#include "drake/automotive/simple_car-inl.h"

#include "drake/common/eigen_autodiff_types.h"

namespace drake {
namespace automotive {

// These instantiations must match the API documentation in simple_car.h.
template class DRAKE_EXPORT SimpleCar<double>;
#if EIGEN_VERSION_AT_LEAST(3, 2, 93)  // True with Drake superbuild.
template class DRAKE_EXPORT SimpleCar<drake::TaylorVarXd>;
#endif

}  // namespace automotive
}  // namespace drake
