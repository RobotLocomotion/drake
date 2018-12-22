#include "pybind11/eval.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/multibody/benchmarks/acrobot/make_acrobot_plant.h"

namespace drake {
namespace pydrake {

using geometry::SceneGraph;
using systems::LeafSystem;

// TODO(eric.cousineau): Bind additional scalar types.
using T = double;

void init_acrobot(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::multibody::benchmarks::acrobot;
  constexpr auto& doc = pydrake_doc.drake.multibody.benchmarks.acrobot;

  py::module::import("pydrake.geometry");
  py::module::import("pydrake.multibody.plant");

  py::class_<AcrobotParameters>(
      m, "AcrobotParameters", doc.AcrobotParameters.doc)
      .def(py::init(), doc.AcrobotParameters.doc);

  m.def("MakeAcrobotPlant",
      py::overload_cast<const AcrobotParameters&, bool, SceneGraph<double>*>(
          &MakeAcrobotPlant),
      py::arg("default_parameters"), py::arg("finalize"),
      py::arg("scene_graph") = nullptr, doc.MakeAcrobotPlant.doc);
}

void init_all(py::module m) {
  py::dict vars = m.attr("__dict__");
  py::exec("from pydrake.multibody.benchmarks.acrobot import *", py::globals(),
      vars);
}

PYBIND11_MODULE(benchmarks, m) {
  init_acrobot(m.def_submodule("acrobot"));
  init_all(m.def_submodule("all"));
}

}  // namespace pydrake
}  // namespace drake
