#include "drake/bindings/pydrake/math/math_py.h"

#include "drake/bindings/pydrake/pydrake_pybind.h"

namespace drake {
namespace pydrake {
namespace {

PYBIND11_MODULE(math, m) {
  // N.B. Docstring contained in `_math_extra.py`.

  py::module::import("pydrake.common");
  py::module::import("pydrake.autodiffutils");
  py::module::import("pydrake.common.eigen_geometry");
  py::module::import("pydrake.symbolic");

  internal::DefineMathMonolith(m);

  ExecuteExtraPythonCode(m, true);
}

}  // namespace
}  // namespace pydrake
}  // namespace drake
