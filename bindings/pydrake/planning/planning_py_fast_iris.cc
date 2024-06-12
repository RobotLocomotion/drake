#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/planning/iris/fast_iris.h"

namespace drake {
namespace pydrake {
namespace internal {

void DefinePlanningFastIris(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::planning;
  constexpr auto& doc = pydrake_doc.drake.planning;

  // FastIrisOptions
  const auto& cls_doc = doc.FastIrisOptions;
  py::class_<FastIrisOptions>(m, "FastIrisOptions", cls_doc.doc)
      .def(py::init<>())
      .def_readwrite("num_particles", &FastIrisOptions::num_particles,
          cls_doc.num_particles.doc)
      .def_readwrite("tau", &FastIrisOptions::tau, cls_doc.tau.doc)
      .def_readwrite("delta", &FastIrisOptions::delta, cls_doc.delta.doc)
      .def_readwrite("admissible_proportion_in_collision",
          &FastIrisOptions::admissible_proportion_in_collision,
          cls_doc.admissible_proportion_in_collision.doc)
      .def_readwrite("containment_points", &FastIrisOptions::containment_points,
          cls_doc.containment_points.doc)
      .def_readwrite("force_containment_points",
          &FastIrisOptions::force_containment_points,
          cls_doc.force_containment_points.doc)
      .def_readwrite("max_iterations", &FastIrisOptions::max_iterations,
          cls_doc.max_iterations.doc)
      .def_readwrite("max_iterations_separating_planes",
          &FastIrisOptions::max_iterations_separating_planes,
          cls_doc.max_iterations_separating_planes.doc)
      .def_readwrite("max_separating_planes_per_iteration",
          &FastIrisOptions::max_separating_planes_per_iteration,
          cls_doc.max_separating_planes_per_iteration.doc)
      .def_readwrite("bisection_steps", &FastIrisOptions::bisection_steps,
          cls_doc.bisection_steps.doc)
      .def_readwrite(
          "parallelize", &FastIrisOptions::parallelize, cls_doc.parallelize.doc)
      .def_readwrite("verbose", &FastIrisOptions::verbose, cls_doc.verbose.doc)
      .def_readwrite("require_sample_point_is_contained",
          &FastIrisOptions::require_sample_point_is_contained,
          cls_doc.require_sample_point_is_contained.doc)
      .def_readwrite("configuration_space_margin",
          &FastIrisOptions::configuration_space_margin,
          cls_doc.configuration_space_margin.doc)
      .def_readwrite("termination_threshold",
          &FastIrisOptions::termination_threshold,
          cls_doc.termination_threshold.doc)
      .def_readwrite("relative_termination_threshold",
          &FastIrisOptions::relative_termination_threshold,
          cls_doc.relative_termination_threshold.doc)
      .def_readwrite(
          "random_seed", &FastIrisOptions::random_seed, cls_doc.random_seed.doc)
      .def_readwrite("mixing_steps", &FastIrisOptions::mixing_steps,
          cls_doc.mixing_steps.doc)
      .def("__repr__", [](const FastIrisOptions& self) {
        return py::str(
            "FastIrisOptions("
            "num_particles={}, "
            "tau={}, "
            "delta={}, "
            "admissible_proportion_in_collision={}, "
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
            .format(self.num_particles, self.tau, self.delta,
                self.admissible_proportion_in_collision,
                self.containment_points, self.force_containment_points,
                self.max_iterations, self.max_iterations_separating_planes,
                self.max_separating_planes_per_iteration, self.bisection_steps,
                self.parallelize, self.verbose,
                self.require_sample_point_is_contained,
                self.configuration_space_margin, self.termination_threshold,
                self.relative_termination_threshold, self.random_seed,
                self.mixing_steps);
      });

  m.def("FastIris", &FastIris, py::arg("checker"),
      py::arg("starting_ellipsoid"), py::arg("domain"),
      py::arg("options") = FastIrisOptions(), doc.FastIris.doc);
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
