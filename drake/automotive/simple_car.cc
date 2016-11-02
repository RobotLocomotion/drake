// NOLINTNEXTLINE(build/include) False positive on inl file.
#include "drake/automotive/simple_car-inl.h"

#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/symbolic_expression.h"

// This is used indirectly to allow DRAKE_ASSERT on symbolic::Expression.
#include "drake/common/symbolic_formula.h"

namespace drake {
namespace automotive {

// These instantiations must match the API documentation in simple_car.h.
template class DRAKE_EXPORT SimpleCar<double>;
#if EIGEN_VERSION_AT_LEAST(3, 2, 93)  // True when built via Drake superbuild.
template class DRAKE_EXPORT SimpleCar<drake::TaylorVarXd>;
#endif
template class DRAKE_EXPORT SimpleCar<drake::symbolic::Expression>;

}  // namespace automotive
}  // namespace drake
