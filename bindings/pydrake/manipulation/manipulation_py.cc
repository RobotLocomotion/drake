#include "drake/bindings/pydrake/manipulation/manipulation_py.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(manipulation, m) {
  PYDRAKE_PREVENT_PYTHON3_MODULE_REIMPORT(m);

  py::module::import("pydrake.systems.framework");

  // The order of these calls matters. Some modules rely on prior definitions.
  internal::DefineManipulationKukaIiwa(m);
  internal::DefineManipulationSchunkWsg(m);
  internal::DefineManipulationUtil(m);

  ExecuteExtraPythonCode(m, true);
}

}  // namespace pydrake
}  // namespace drake
