#include "drake/bindings/pydrake/symbolic/symbolic_py.h"

#include "drake/bindings/pydrake/pydrake_pybind.h"

namespace drake {
namespace pydrake {
namespace {

PYBIND11_MODULE(symbolic, m) {
  m.doc() =
      "Symbolic variable, variables, monomial, expression, polynomial, and "
      "formula";

  py::module::import("pydrake.common");

  // Install NumPy warning filtres.
  // N.B. This may interfere with other code, but until that is a confirmed
  // issue, we should aggressively try to avoid these warnings.
  py::module::import("pydrake.common.deprecation")
      .attr("install_numpy_warning_filters")();

  // Install NumPy formatters patch.
  py::module::import("pydrake.common.compatibility")
      .attr("maybe_patch_numpy_formatters")();

  internal::DefineSymbolicMonolith(m);

  ExecuteExtraPythonCode(m, true);
}

}  // namespace
}  // namespace pydrake
}  // namespace drake
