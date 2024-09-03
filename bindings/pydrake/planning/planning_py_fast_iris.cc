#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/planning/planning_py.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/planning/iris/fast_iris.h"

namespace drake {
namespace pydrake {
namespace internal {

void DefinePlanningIrisZO(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::planning;
  constexpr auto& doc = pydrake_doc.drake.planning;

  // IrisZOOptions
  const auto& cls_doc = doc.IrisZOOptions;
  py::class_<IrisZOOptions>(m, "IrisZOOptions", cls_doc.doc)
      .def(py::init<>())
      .def_readwrite("num_particles", &IrisZOOptions::num_particles,
          cls_doc.num_particles.doc)
      .def_readwrite("tau", &IrisZOOptions::tau, cls_doc.tau.doc)
      .def_readwrite("delta", &IrisZOOptions::delta, cls_doc.delta.doc)
      .def_readwrite("epsilon", &IrisZOOptions::epsilon, cls_doc.epsilon.doc)
      .def_readwrite("containment_points", &IrisZOOptions::containment_points,
          cls_doc.containment_points.doc)
      .def_readwrite("force_containment_points",
          &IrisZOOptions::force_containment_points,
          cls_doc.force_containment_points.doc)
      .def_readwrite("max_iterations", &IrisZOOptions::max_iterations,
          cls_doc.max_iterations.doc)
      .def_readwrite("max_iterations_separating_planes",
          &IrisZOOptions::max_iterations_separating_planes,
          cls_doc.max_iterations_separating_planes.doc)
      .def_readwrite("max_separating_planes_per_iteration",
          &IrisZOOptions::max_separating_planes_per_iteration,
          cls_doc.max_separating_planes_per_iteration.doc)
      .def_readwrite("bisection_steps", &IrisZOOptions::bisection_steps,
          cls_doc.bisection_steps.doc)
      .def_readwrite(
          "parallelize", &IrisZOOptions::parallelize, cls_doc.parallelize.doc)
      .def_readwrite("verbose", &IrisZOOptions::verbose, cls_doc.verbose.doc)
      .def_readwrite("require_sample_point_is_contained",
          &IrisZOOptions::require_sample_point_is_contained,
          cls_doc.require_sample_point_is_contained.doc)
      .def_readwrite("configuration_space_margin",
          &IrisZOOptions::configuration_space_margin,
          cls_doc.configuration_space_margin.doc)
      .def_readwrite("termination_threshold",
          &IrisZOOptions::termination_threshold,
          cls_doc.termination_threshold.doc)
      .def_readwrite("relative_termination_threshold",
          &IrisZOOptions::relative_termination_threshold,
          cls_doc.relative_termination_threshold.doc)
      .def_readwrite(
          "random_seed", &IrisZOOptions::random_seed, cls_doc.random_seed.doc)
      .def_readwrite("mixing_steps", &IrisZOOptions::mixing_steps,
          cls_doc.mixing_steps.doc)
      .def("__repr__", [](const IrisZOOptions& self) {
        return py::str(
            "IrisZOOptions("
            "num_particles={}, "
            "tau={}, "
            "delta={}, "
            "epsilon={}, "
            "force_containment_points={}, "
            "num_consecutive_failures={}, "
            "max_iterations={}, "
            "max_iterations_separating_planes={}, "
            "max_separating_planes_per_iteration={}, "
            "bisection_steps={}, "
            "parallelize={}, "
            "verbose={}, "
            "require_sample_point_is_contained={}, "
            "configuration_space_margin={}, "
            "termination_threshold={}, "
            "relative_termination_threshold={}, "
            "random_seed={}, "
            "mixing_steps={}, "
            ")")
            .format(self.num_particles, self.tau, self.delta, self.epsilon,
                self.containment_points, self.force_containment_points,
                self.max_iterations, self.max_iterations_separating_planes,
                self.max_separating_planes_per_iteration, self.bisection_steps,
                self.parallelize, self.verbose,
                self.require_sample_point_is_contained,
                self.configuration_space_margin, self.termination_threshold,
                self.relative_termination_threshold, self.random_seed,
                self.mixing_steps);
      });

  m.def("IrisZO", &IrisZO, py::arg("checker"), py::arg("starting_ellipsoid"),
      py::arg("domain"), py::arg("options") = IrisZOOptions(), doc.IrisZO.doc);
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
