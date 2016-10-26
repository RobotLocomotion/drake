#include "drake/automotive/simple_car-inl.h"

#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/symbolic_expression.h"
#include "drake/common/symbolic_formula.h"

namespace drake {
namespace automotive {

// These instantiations must match the API documentation in simple_car.h.
template class DRAKE_EXPORT SimpleCar<double>;
template class DRAKE_EXPORT SimpleCar<drake::TaylorVarXd>;
template class DRAKE_EXPORT SimpleCar<drake::symbolic::Expression>;

}  // namespace automotive
}  // namespace drake
