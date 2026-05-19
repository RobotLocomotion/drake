#include "drake/bindings/pydrake/visualization/visualization_py.h"

namespace drake {
namespace pydrake {

PYDRAKE_MODULE(visualization, m) {
  PYDRAKE_PREVENT_PYTHON3_MODULE_REIMPORT(m);

  m.doc() = R"""(
Bindings for Visualization.
)""";

  py::module_::import_("pydrake.geometry");
  py::module_::import_("pydrake.multibody");
  py::module_::import_("pydrake.systems");

  // The order of these calls matters. Some modules rely on prior definitions.
  internal::DefineVisualizationConfig(m);
  internal::DefineVisualizationImageSystems(m);
  internal::DefineVisualizationSliders(m);

  py::module_::import_("pydrake.visualization._meldis");
  py::module_::import_("pydrake.visualization._model_visualizer");
  ExecuteExtraPythonCode(m, true);
}

}  // namespace pydrake
}  // namespace drake
