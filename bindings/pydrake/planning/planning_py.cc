#include "drake/bindings/pydrake/planning/planning_py.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(planning, m) {
  PYDRAKE_PREVENT_PYTHON3_MODULE_REIMPORT(m);

  m.doc() = R"""(
XXX
)""";

  // py::module::import("pydrake.autodiffutils");
  // py::module::import("pydrake.common.value");
  // py::module::import("pydrake.symbolic");

  // The order of these calls matters. Some modules rely on prior definitions.
  internal::DefinePlanningRobotDiagram(m);
}

}  // namespace pydrake
}  // namespace drake
