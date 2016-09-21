#include "drake/automotive/simple_car-inl.h"

#include "drake/common/eigen_autodiff_types.h"

namespace drake {
namespace automotive {

// These instantiations must match the API documentation in simple_car.h.
template class DRAKEAUTOMOTIVE_EXPORT SimpleCar<double>;
template class DRAKEAUTOMOTIVE_EXPORT SimpleCar<drake::TaylorVarXd>;

}  // namespace automotive
}  // namespace drake
