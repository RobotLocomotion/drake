//#include "drake/bindings/pydrake/documentation_pybind.h"
//#include "drake/planning/graph_algorithms/max_clique.h"
//#include "drake/bindings/pydrake/pydrake_pybind.h"
//
//
//namespace drake {
//namespace pydrake {
//namespace internal {
//
//void DefinePlanningGraphAlgorithms(py::module m) {
//  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
//  using namespace drake::planning::graph_algorithms;
//  constexpr auto& doc = pydrake_doc.drake.planning.graph_algorithms;
//  {
//    using Class = MaxCliqueSolverBase;
//    m.def("MaxCliqueSolverBase", &planning::, py::arg("checker"),
//          py::arg("points"), py::arg("parallelize") = true,
//          doc.VisibilityGraph.doc);
//  }
//}
//
//
//}  // namespace internal
//}  // namespace pydrake
//}  // namespace drake