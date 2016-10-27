#include "drake/automotive/simple_car-inl.h"

#include "drake/common/drake_export.h"
#include "drake/common/symbolic_expression.h"
// This is used indirectly to allow DRAKE_ASSERT on symbolic::Expression.
#include "drake/common/symbolic_formula.h"

namespace drake {
namespace automotive {

// This instantiation must match the API documentation in simple_car.h.
template class DRAKE_EXPORT SimpleCar<drake::symbolic::Expression>;

}  // namespace automotive
}  // namespace drake
