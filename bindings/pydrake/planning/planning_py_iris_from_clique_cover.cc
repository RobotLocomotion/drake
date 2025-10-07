#include <vector>

#include "drake/bindings/generated_docstrings/planning_iris.h"
#include "drake/bindings/pydrake/planning/planning_py.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/planning/graph_algorithms/max_clique_solver_base.h"
#include "drake/planning/iris/iris_from_clique_cover.h"

namespace drake {
namespace pydrake {
namespace internal {

void DefinePlanningIrisFromCliqueCover(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::planning;
  constexpr auto& doc = pydrake_doc_planning_iris.drake.planning;
  auto cls_doc = doc.IrisFromCliqueCoverOptions;
  py::class_<IrisFromCliqueCoverOptions>(
      m, "IrisFromCliqueCoverOptions", cls_doc.doc)
      .def(py::init<>())
      .def_readwrite("iris_options", &IrisFromCliqueCoverOptions::iris_options,
          cls_doc.iris_options.doc)
      .def_readwrite("coverage_termination_threshold",
          &IrisFromCliqueCoverOptions::coverage_termination_threshold,
          cls_doc.coverage_termination_threshold.doc)
      .def_readwrite("iteration_limit",
          &IrisFromCliqueCoverOptions::iteration_limit,
          cls_doc.iteration_limit.doc)
      .def_readwrite("num_points_per_coverage_check",
          &IrisFromCliqueCoverOptions::num_points_per_coverage_check,
          cls_doc.num_points_per_coverage_check.doc)
      .def_readwrite("parallelism", &IrisFromCliqueCoverOptions::parallelism,
          cls_doc.parallelism.doc)
      .def_readwrite("minimum_clique_size",
          &IrisFromCliqueCoverOptions::minimum_clique_size,
          cls_doc.minimum_clique_size.doc)
      .def_readwrite("num_points_per_visibility_round",
          &IrisFromCliqueCoverOptions::num_points_per_visibility_round,
          cls_doc.num_points_per_visibility_round.doc)
      .def_readwrite("rank_tol_for_minimum_volume_circumscribed_ellipsoid",
          &IrisFromCliqueCoverOptions::
              rank_tol_for_minimum_volume_circumscribed_ellipsoid,
          cls_doc.rank_tol_for_minimum_volume_circumscribed_ellipsoid.doc)
      .def_readwrite("point_in_set_tol",
          &IrisFromCliqueCoverOptions::point_in_set_tol,
          cls_doc.point_in_set_tol.doc);

  m.def(
      "IrisInConfigurationSpaceFromCliqueCover",
      [](const CollisionChecker& checker,
          const IrisFromCliqueCoverOptions& options, RandomGenerator generator,
          std::vector<geometry::optimization::HPolyhedron> sets,
          const planning::graph_algorithms::MaxCliqueSolverBase*
              max_clique_solver) {
        IrisInConfigurationSpaceFromCliqueCover(
            checker, options, &generator, &sets, max_clique_solver);
        return sets;
      },
      py::arg("checker"), py::arg("options"), py::arg("generator"),
      py::arg("sets"), py::arg("max_clique_solver") = nullptr,
      py::call_guard<py::gil_scoped_release>(),
      doc.IrisInConfigurationSpaceFromCliqueCover.doc);
}  // DefinePlanningIrisFromCliqueCover
}  // namespace internal
}  // namespace pydrake
}  // namespace drake
