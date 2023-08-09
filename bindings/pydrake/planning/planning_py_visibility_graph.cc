#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/planning/iris_regions_from_clique_cover.h"
#include "drake/planning/visibility_graph.h"

namespace drake {
namespace pydrake {
namespace internal {

void DefinePlanningVisibilityGraph(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::planning;
  constexpr auto& doc = pydrake_doc.drake.planning;

  m.def("VisibilityGraph", &planning::VisibilityGraph, py::arg("checker"),
      py::arg("points"), py::arg("parallelize") = true,
      doc.VisibilityGraph.doc);

  m.def("IrisRegionsFromCliqueCover", &planning::IrisRegionsFromCliqueCover,
      py::arg("checker"), py::arg("points"),
      py::arg("options") = geometry::optimization::IrisOptions(),
      py::arg("minimum_clique_size") = 3, py::arg("parallelize") = true,
      doc.IrisRegionsFromCliqueCover.doc);
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
