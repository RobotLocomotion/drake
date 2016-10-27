#include "drake/automotive/simple_car-inl.h"

#include "drake/common/drake_export.h"
#include "drake/common/eigen_autodiff_types.h"

namespace drake {
namespace automotive {

// This instantiation must match the API documentation in simple_car.h.
#if EIGEN_VERSION_AT_LEAST(3, 2, 93)  // True with Drake superbuild.
template class DRAKE_EXPORT SimpleCar<drake::TaylorVarXd>;
#endif

}  // namespace automotive
}  // namespace drake
