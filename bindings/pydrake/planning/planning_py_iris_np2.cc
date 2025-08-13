#include "drake/bindings/pydrake/common/wrap_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
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
  constexpr auto& doc = pydrake_doc.drake.planning;

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
          ray_sampler_options_doc.num_particles_to_walk_towards.doc);

  // IrisNp2Options
  const auto& cls_doc = doc.IrisNp2Options;
  py::class_<IrisNp2Options> iris_np2_options(m, "IrisNp2Options", cls_doc.doc);
  iris_np2_options.def(py::init<>())
      .def_readwrite("sampled_iris_options",
          &IrisNp2Options::sampled_iris_options,
          cls_doc.sampled_iris_options.doc)
      .def_readwrite("parameterization", &IrisNp2Options::parameterization,
          cls_doc.parameterization.doc)
      .def_readwrite("sampling_strategy", &IrisNp2Options::sampling_strategy,
          cls_doc.sampling_strategy.doc)
      .def_readwrite("ray_sampler_options",
          &IrisNp2Options::ray_sampler_options, cls_doc.ray_sampler_options.doc)
      .def("__repr__", [](const IrisNp2Options& self) {
        return py::str(
            "IrisNp2Options("
            "sampled_iris_options={}, "
            ")")
            .format(self.sampled_iris_options);
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
