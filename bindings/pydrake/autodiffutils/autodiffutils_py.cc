#include "drake/bindings/pydrake/autodiffutils/autodiffutils_py.h"

#include "drake/bindings/pydrake/pydrake_pybind.h"

namespace drake {
namespace pydrake {
namespace {

PYBIND11_MODULE(autodiffutils, m) {
  m.doc() = "Bindings for Eigen AutoDiff Scalars";

  py::module::import("pydrake.common");

  // Install NumPy warning filtres.
  // N.B. This may interfere with other code, but until that is a confirmed
  // issue, we should aggressively try to avoid these warnings.
  py::module::import("pydrake.common.deprecation")
      .attr("install_numpy_warning_filters")();

  internal::DefineAutodiffutils(m);

  ExecuteExtraPythonCode(m, true);
}

}  // namespace
}  // namespace pydrake
}  // namespace drake
