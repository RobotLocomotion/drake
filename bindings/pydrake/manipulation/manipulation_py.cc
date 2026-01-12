#include "drake/bindings/pydrake/manipulation/manipulation_py.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(manipulation, m) {
  PYDRAKE_PREVENT_PYTHON3_MODULE_REIMPORT(m);

  py::module::import("pydrake.multibody.parsing");
  py::module::import("pydrake.systems.framework");
  py::module::import("pydrake.systems.primitives");

  // The order of these calls matters. Some modules rely on prior definitions.
  internal::DefineManipulationKukaIiwa(m);
  internal::DefineManipulationFrankaPanda(m);
  internal::DefineManipulationSchunkWsg(m);
  internal::DefineManipulationUtil(m);

  ExecuteExtraPythonCode(m, true);
}

}  // namespace pydrake
}  // namespace drake
