#include <utility>

#include "drake/bindings/generated_docstrings/planning_iris.h"
#include "drake/bindings/pydrake/common/wrap_pybind.h"
#include "drake/bindings/pydrake/planning/planning_py.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/symbolic_types_pybind.h"
#include "drake/planning/iris/iris_np2.h"

namespace drake {
namespace pydrake {
namespace internal {

void DefinePlanningIrisNp2(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::planning;
  constexpr auto& doc = pydrake_doc_planning_iris.drake.planning;

  // RaySamplerOptions
  const auto& ray_sampler_options_doc = doc.RaySamplerOptions;
  py::class_<RaySamplerOptions> ray_sampler_options(
      m, "RaySamplerOptions", ray_sampler_options_doc.doc);
  ray_sampler_options.def(py::init<>())
      .def_readwrite("only_walk_toward_collisions",
          &RaySamplerOptions::only_walk_toward_collisions,
          ray_sampler_options_doc.only_walk_toward_collisions.doc)
      .def_readwrite("ray_search_num_steps",
          &RaySamplerOptions::ray_search_num_steps,
          ray_sampler_options_doc.ray_search_num_steps.doc)
      .def_readwrite("num_particles_to_walk_towards",
          &RaySamplerOptions::num_particles_to_walk_towards,
          ray_sampler_options_doc.num_particles_to_walk_towards.doc)
      .def("__repr__", [](const RaySamplerOptions& self) {
        return py::str(
            "RaySamplerOptions("
            "only_walk_toward_collisions={}, "
            "ray_search_num_steps={}, "
            "num_particles_to_walk_towards={}, "
            ")")
            .format(self.only_walk_toward_collisions, self.ray_search_num_steps,
                self.num_particles_to_walk_towards);
      });

  // IrisNp2Options
  const auto& cls_doc = doc.IrisNp2Options;
  py::class_<IrisNp2Options> iris_np2_options(m, "IrisNp2Options", cls_doc.doc);
  iris_np2_options.def(py::init<>())
      .def_property("solver_options",
          py::cpp_function(
              [](IrisNp2Options& self) { return &(self.solver_options); },
              py_rvp::reference_internal),
          py::cpp_function(
              [](IrisNp2Options& self, solvers::SolverOptions solver_options) {
                self.solver_options = std::move(solver_options);
              }),
          cls_doc.solver_options.doc)
      .def_readwrite("sampled_iris_options",
          &IrisNp2Options::sampled_iris_options,
          cls_doc.sampled_iris_options.doc)
      .def_readwrite("parameterization", &IrisNp2Options::parameterization,
          cls_doc.parameterization.doc)
      .def_readwrite("sampling_strategy", &IrisNp2Options::sampling_strategy,
          cls_doc.sampling_strategy.doc)
      .def_readwrite("ray_sampler_options",
          &IrisNp2Options::ray_sampler_options, cls_doc.ray_sampler_options.doc)
      .def_readwrite("add_hyperplane_if_solve_fails",
          &IrisNp2Options::add_hyperplane_if_solve_fails,
          cls_doc.add_hyperplane_if_solve_fails.doc)
      .def("__repr__", [](const IrisNp2Options& self) {
        return py::str(
            "IrisNp2Options("
            "sampled_iris_options={}, "
            "sampling_strategy={}, "
            "ray_sampler_options={}, "
            "add_hyperplane_if_solve_fails={}, "
            ")")
            .format(self.sampled_iris_options, self.sampling_strategy,
                self.ray_sampler_options, self.add_hyperplane_if_solve_fails);
      });

  DefReadWriteKeepAlive(
      &iris_np2_options, "solver", &IrisNp2Options::solver, cls_doc.solver.doc);

  // The `options` contains a `Parallelism`; we must release the GIL.
  m.def("IrisNp2", &IrisNp2, py::arg("checker"), py::arg("starting_ellipsoid"),
      py::arg("domain"), py::arg("options") = IrisNp2Options(),
      py::call_guard<py::gil_scoped_release>(), doc.IrisNp2.doc);
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
