#include "drake/bindings/pydrake/visualization/visualization_py.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(visualization, m) {
  PYDRAKE_PREVENT_PYTHON3_MODULE_REIMPORT(m);

  m.doc() = R"""(
Bindings for Visualization.
)""";

  py::module::import("pydrake.geometry");
  py::module::import("pydrake.multibody");
  py::module::import("pydrake.systems");

  // The order of these calls matters. Some modules rely on prior definitions.
  internal::DefineVisualizationConfig(m);
  internal::DefineVisualizationImageSystems(m);
  internal::DefineVisualizationSliders(m);

  py::module::import("pydrake.visualization.meldis");
  py::module::import("pydrake.visualization.model_visualizer");
  ExecuteExtraPythonCode(m, true);
}

}  // namespace pydrake
}  // namespace drake
