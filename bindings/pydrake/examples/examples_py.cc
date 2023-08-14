#include "drake/bindings/pydrake/examples/examples_py.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(examples, m) {
  PYDRAKE_PREVENT_PYTHON3_MODULE_REIMPORT(m);

  m.doc() = R"""(
Provides bindings of existing C++ example library code as well as a few pure
Python examples.
)""";

  py::module::import("pydrake.geometry");
  py::module::import("pydrake.multibody.plant");
  py::module::import("pydrake.systems.framework");
  py::module::import("pydrake.systems.primitives");
  py::module::import("pydrake.systems.sensors");

  // These are in alphabetical order; the modules do not depend on each other.
  internal::DefineExamplesAcrobot(m);
  internal::DefineExamplesCompassGait(m);
  internal::DefineExamplesManipulationStation(m);
  internal::DefineExamplesPendulum(m);
  internal::DefineExamplesQuadrotor(m);
  internal::DefineExamplesRimlessWheel(m);
  internal::DefineExamplesVanDerPol(m);

  const bool use_subdir = true;
  ExecuteExtraPythonCode(m, use_subdir);
}

}  // namespace pydrake
}  // namespace drake
