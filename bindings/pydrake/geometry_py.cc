#include "drake/bindings/pydrake/geometry_py.h"

#include "pybind11/eval.h"

#include "drake/bindings/pydrake/pydrake_pybind.h"

namespace drake {
namespace pydrake {
namespace {

void def_geometry_all(py::module m) {
  py::dict vars = m.attr("__dict__");
  py::exec(
      "from pydrake.geometry import *\n"
      "from pydrake.geometry.render import *\n"
      "from pydrake.geometry.optimization import *\n",
      py::globals(), vars);
}
}  // namespace

PYBIND11_MODULE(geometry, m) {
  PYDRAKE_PREVENT_PYTHON3_MODULE_REIMPORT(m);
  py::module::import("pydrake.math");

  /* The order of execution matters -- a module may rely on the definition
   of bindings executed prior to it. */
  DefineGeometryCommon(m);
  DefineGeometryHydro(m);
  DefineGeometryRender(m.def_submodule("render"));
  DefineGeometrySceneGraph(m);
  DefineGeometryOptimization(m.def_submodule("optimization"));
  DefineGeometryVisualizers(m);
  def_geometry_all(m.def_submodule("all"));
}

}  // namespace pydrake
}  // namespace drake
