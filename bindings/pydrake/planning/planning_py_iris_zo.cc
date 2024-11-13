#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/planning/planning_py.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/planning/iris/iris_zo.h"

namespace drake {
namespace pydrake {
namespace internal {

void DefinePlanningIrisZo(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::planning;
  constexpr auto& doc = pydrake_doc.drake.planning;

  // IrisZoOptions
  const auto& cls_doc = doc.IrisZoOptions;
  py::class_<IrisZoOptions>(m, "IrisZoOptions", cls_doc.doc)
      .def(py::init<>())
      .def_readwrite("num_particles", &IrisZoOptions::num_particles,
          cls_doc.num_particles.doc)
      .def_readwrite("tau", &IrisZoOptions::tau, cls_doc.tau.doc)
      .def_readwrite("delta", &IrisZoOptions::delta, cls_doc.delta.doc)
      .def_readwrite("epsilon", &IrisZoOptions::epsilon, cls_doc.epsilon.doc)
      .def_readwrite("containment_points", &IrisZoOptions::containment_points,
          cls_doc.containment_points.doc)
      .def_readwrite("force_containment_points",
          &IrisZoOptions::force_containment_points,
          cls_doc.force_containment_points.doc)
      .def_readwrite("max_iterations", &IrisZoOptions::max_iterations,
          cls_doc.max_iterations.doc)
      .def_readwrite("max_iterations_separating_planes",
          &IrisZoOptions::max_iterations_separating_planes,
          cls_doc.max_iterations_separating_planes.doc)
      .def_readwrite("max_separating_planes_per_iteration",
          &IrisZoOptions::max_separating_planes_per_iteration,
          cls_doc.max_separating_planes_per_iteration.doc)
      .def_readwrite("bisection_steps", &IrisZoOptions::bisection_steps,
          cls_doc.bisection_steps.doc)
      .def_readwrite(
          "parallelize", &IrisZoOptions::parallelize, cls_doc.parallelize.doc)
      .def_readwrite("verbose", &IrisZoOptions::verbose, cls_doc.verbose.doc)
      .def_readwrite("require_sample_point_is_contained",
          &IrisZoOptions::require_sample_point_is_contained,
          cls_doc.require_sample_point_is_contained.doc)
      .def_readwrite("configuration_space_margin",
          &IrisZoOptions::configuration_space_margin,
          cls_doc.configuration_space_margin.doc)
      .def_readwrite("termination_threshold",
          &IrisZoOptions::termination_threshold,
          cls_doc.termination_threshold.doc)
      .def_readwrite("relative_termination_threshold",
          &IrisZoOptions::relative_termination_threshold,
          cls_doc.relative_termination_threshold.doc)
      .def_readwrite(
          "random_seed", &IrisZoOptions::random_seed, cls_doc.random_seed.doc)
      .def_readwrite("mixing_steps", &IrisZoOptions::mixing_steps,
          cls_doc.mixing_steps.doc)
      .def("__repr__", [](const IrisZoOptions& self) {
        return py::str(
            "IrisZoOptions("
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

  m.def("IrisZo", &IrisZo, py::arg("checker"), py::arg("starting_ellipsoid"),
      py::arg("doma din"), py::arg("options") = IrisZoOptions(),
      doc.IrisZo.doc);
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
