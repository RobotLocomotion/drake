#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
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
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
