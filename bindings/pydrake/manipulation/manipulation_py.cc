#include "drake/bindings/pydrake/manipulation/manipulation_py.h"

namespace drake {
namespace pydrake {

PYDRAKE_MODULE(manipulation, m) {
  PYDRAKE_PREVENT_PYTHON3_MODULE_REIMPORT(m);

  py::module_::import_("pydrake.multibody.parsing");
  py::module_::import_("pydrake.systems.framework");
  py::module_::import_("pydrake.systems.primitives");

  // The order of these calls matters. Some modules rely on prior definitions.
  internal::DefineManipulationKukaIiwa(m);
  internal::DefineManipulationFrankaPanda(m);
  internal::DefineManipulationSchunkWsg(m);
  internal::DefineManipulationUtil(m);

  ExecuteExtraPythonCode(m, true);
}

}  // namespace pydrake
}  // namespace drake
