#include "drake/bindings/pydrake/autodiff_types_pybind.h"
#include "drake/bindings/pydrake/math_operators_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/symbolic_types_pybind.h"

namespace drake {
namespace pydrake {
namespace internal {

void DefineMathOperators(py::module m) {
  // Define math operations for all three scalar types.
  pydrake::internal::BindMathOperators<double>(&m);
  pydrake::internal::BindMathOperators<AutoDiffXd>(&m);
  pydrake::internal::BindMathOperators<symbolic::Expression>(&m);
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
