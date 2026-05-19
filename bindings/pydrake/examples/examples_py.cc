#include "drake/bindings/pydrake/examples/examples_py.h"

namespace drake {
namespace pydrake {

PYDRAKE_MODULE(examples, m) {
  PYDRAKE_PREVENT_PYTHON3_MODULE_REIMPORT(m);

  m.doc() = R"""(
Provides bindings of existing C++ example library code as well as a few pure
Python examples.
)""";

  py::module_::import_("pydrake.geometry");
  py::module_::import_("pydrake.multibody.plant");
  py::module_::import_("pydrake.systems.framework");
  py::module_::import_("pydrake.systems.primitives");
  py::module_::import_("pydrake.systems.sensors");

  // These are in alphabetical order; the modules do not depend on each other.
  internal::DefineExamplesAcrobot(m);
  internal::DefineExamplesCompassGait(m);
  internal::DefineExamplesPendulum(m);
  internal::DefineExamplesQuadrotor(m);
  internal::DefineExamplesRimlessWheel(m);
  internal::DefineExamplesVanDerPol(m);
}

}  // namespace pydrake
}  // namespace drake
