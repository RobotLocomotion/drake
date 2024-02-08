#include "drake/bindings/pydrake/common/cpp_template_pybind.h"
#include "drake/bindings/pydrake/common/wrap_function.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/geometry/optimization_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/geometry/optimization/hyperrectangle.h"
#include "drake/planning/iris_from_clique_cover.h"

namespace drake {
namespace pydrake {
namespace internal {

void DefinePlanningIrisFromCliqueCover(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::planning;
  constexpr auto& doc = pydrake_doc.drake.planning;
  auto cls_doc = doc.IrisFromCliqueCoverOptions;
  py::class_<IrisFromCliqueCoverOptions>(
      m, "IrisFromCliqueCoverOptions", cls_doc.doc)
      .def(py::init<>())
      // A Python specific constructor to allow the clique cover solver to be
      // set.
      .def(py::init([](std::unique_ptr<
                        planning::graph_algorithms::MaxCliqueSolverBase>
                            solver) {
        IrisFromCliqueCoverOptions ret{};
        ret.max_clique_solver = std::move(solver);
        return ret;
      }),
          py::arg("solver"))
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
      .def_property_readonly(
          "max_clique_solver",
          [](const IrisFromCliqueCoverOptions& self) {
            return self.max_clique_solver.get();
          },
          py_rvp::reference_internal)
      .def_readwrite("rank_tol_for_lowner_john_ellipse",
          &IrisFromCliqueCoverOptions::rank_tol_for_lowner_john_ellipse,
          cls_doc.rank_tol_for_lowner_john_ellipse.doc)
      .def_readwrite("point_in_set_tol",
          &IrisFromCliqueCoverOptions::point_in_set_tol,
          cls_doc.point_in_set_tol.doc);

  m.def(
      "IrisInConfigurationSpaceFromCliqueCover",
      [](const CollisionChecker& checker,
          const IrisFromCliqueCoverOptions& options, RandomGenerator generator,
          std::vector<geometry::optimization::HPolyhedron> sets) {
        IrisInConfigurationSpaceFromCliqueCover(
            checker, options, &generator, &sets);
        return sets;
      },
      py::arg("checker"), py::arg("options"), py::arg("generator"),
      py::arg("sets"), doc.IrisInConfigurationSpaceFromCliqueCover.doc);
}  // DefinePlanningIrisFromCliqueCover
}  // namespace internal
}  // namespace pydrake
}  // namespace drake
