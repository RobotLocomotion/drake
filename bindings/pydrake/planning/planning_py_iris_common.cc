#include <string>

#include "drake/bindings/generated_docstrings/planning_iris.h"
#include "drake/bindings/pydrake/autodiff_types_pybind.h"
#include "drake/bindings/pydrake/common/cpp_template_pybind.h"
#include "drake/bindings/pydrake/common/wrap_pybind.h"
#include "drake/bindings/pydrake/planning/planning_py.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/symbolic_types_pybind.h"
#include "drake/planning/iris/iris_common.h"

namespace drake {
namespace pydrake {
namespace {

void DefinePlanningCommonSampledIrisOptions(py::module_ m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::planning;
  constexpr auto& doc = pydrake_doc_planning_iris.drake.planning;

  // CommonSampledIrisOptions
  const auto& cls_doc = doc.CommonSampledIrisOptions;
  class_<CommonSampledIrisOptions> common_sampled_iris_options(
      m, "CommonSampledIrisOptions", cls_doc.doc);
  common_sampled_iris_options  // BR
      .def(py::init<>())
      .def_rw("num_particles", &CommonSampledIrisOptions::num_particles,
          cls_doc.num_particles.doc)
      .def_rw("tau", &CommonSampledIrisOptions::tau, cls_doc.tau.doc)
      .def_rw("delta", &CommonSampledIrisOptions::delta, cls_doc.delta.doc)
      .def_rw(
          "epsilon", &CommonSampledIrisOptions::epsilon, cls_doc.epsilon.doc)
      .def_rw("containment_points",
          &CommonSampledIrisOptions::containment_points,
          cls_doc.containment_points.doc)
      .def_rw("max_iterations", &CommonSampledIrisOptions::max_iterations,
          cls_doc.max_iterations.doc)
      .def_rw("max_iterations_separating_planes",
          &CommonSampledIrisOptions::max_iterations_separating_planes,
          cls_doc.max_iterations_separating_planes.doc)
      .def_rw("max_separating_planes_per_iteration",
          &CommonSampledIrisOptions::max_separating_planes_per_iteration,
          cls_doc.max_separating_planes_per_iteration.doc)
      .def_rw("parallelism", &CommonSampledIrisOptions::parallelism,
          cls_doc.parallelism.doc)
      .def_rw(
          "verbose", &CommonSampledIrisOptions::verbose, cls_doc.verbose.doc)
      .def_rw("require_sample_point_is_contained",
          &CommonSampledIrisOptions::require_sample_point_is_contained,
          cls_doc.require_sample_point_is_contained.doc)
      .def_rw("configuration_space_margin",
          &CommonSampledIrisOptions::configuration_space_margin,
          cls_doc.configuration_space_margin.doc)
      .def_rw("relax_margin", &CommonSampledIrisOptions::relax_margin,
          cls_doc.relax_margin.doc)
      .def_rw("termination_threshold",
          &CommonSampledIrisOptions::termination_threshold,
          cls_doc.termination_threshold.doc)
      .def_rw("relative_termination_threshold",
          &CommonSampledIrisOptions::relative_termination_threshold,
          cls_doc.relative_termination_threshold.doc)
      .def_rw("remove_all_collisions_possible",
          &CommonSampledIrisOptions::remove_all_collisions_possible,
          cls_doc.remove_all_collisions_possible.doc)
      .def_rw("random_seed", &CommonSampledIrisOptions::random_seed,
          cls_doc.random_seed.doc)
      .def_rw("mixing_steps", &CommonSampledIrisOptions::mixing_steps,
          cls_doc.mixing_steps.doc)
      .def_rw("sample_particles_in_parallel",
          &CommonSampledIrisOptions::sample_particles_in_parallel,
          cls_doc.sample_particles_in_parallel.doc)
      .def("__repr__", [](const CommonSampledIrisOptions& self) {
        return py::str(
            "CommonSampledIrisOptions("
            "num_particles={}, "
            "tau={}, "
            "delta={}, "
            "epsilon={}, "
            "max_iterations={}, "
            "max_iterations_separating_planes={}, "
            "max_separating_planes_per_iteration={}, "
            "parallelism={}, "
            "verbose={}, "
            "require_sample_point_is_contained={}, "
            "configuration_space_margin={}, "
            "relax_margin={}, "
            "termination_threshold={}, "
            "relative_termination_threshold={}, "
            "remove_all_collisions_possible={}, "
            "random_seed={}, "
            "mixing_steps={}, "
            "sample_particles_in_parallel={}, "
            ")")
            .format(self.num_particles, self.tau, self.delta, self.epsilon,
                self.max_iterations, self.max_iterations_separating_planes,
                self.max_separating_planes_per_iteration, self.parallelism,
                self.verbose, self.require_sample_point_is_contained,
                self.configuration_space_margin, self.relax_margin,
                self.termination_threshold, self.relative_termination_threshold,
                self.remove_all_collisions_possible, self.random_seed,
                self.mixing_steps, self.sample_particles_in_parallel);
      });

  DefReadWriteKeepAlive(&common_sampled_iris_options,
      "prog_with_additional_constraints",
      &CommonSampledIrisOptions::prog_with_additional_constraints,
      cls_doc.prog_with_additional_constraints.doc);
}

void DefinePlanningIrisParameterizationFunction(py::module_ m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::planning;
  constexpr auto& doc = pydrake_doc_planning_iris.drake.planning;

  // IrisParameterizationFunction
  const auto& cls_doc = doc.IrisParameterizationFunction;

  const std::string parameterization_function_docstring = R"(
Constructor for when the user provides a callable function to be used as the
parameterization. ``parameterization`` is the function itself and ``dimension`` 
is the input dimension.

.. note:: In order to use IrisNp2, the user-provided parameterization function
   must support double and AutoDiffXd. If it does not support AutoDiffXd, then
   only IrisZo can be used.

.. note:: Due to the GIL, it is inefficient to call a Python function
   concurrently across multiple C++ threads. Therefore, the constructor
   automatically sets threadsafe to false.
)";

  class_<IrisParameterizationFunction> iris_parameterization_function(
      m, "IrisParameterizationFunction", cls_doc.doc);
  iris_parameterization_function  // BR
      .def(py::init<>(), cls_doc.ctor.doc_0args)
      .def(py::init([](const py::callable& parameterization,
                        int parameterization_dimension) {
        return IrisParameterizationFunction(
            py::cast<
                IrisParameterizationFunction::ParameterizationFunctionDouble>(
                parameterization),
            py::cast<
                IrisParameterizationFunction::ParameterizationFunctionAutodiff>(
                parameterization),
            /* parameterization_is_threadsafe = */ false,
            parameterization_dimension);
      }),
          py::arg("parameterization"), py::arg("dimension"),
          parameterization_function_docstring.c_str())
      .def(py::init<const Eigen::VectorX<symbolic::Expression>&,
               const Eigen::VectorX<symbolic::Variable>&>(),
          py::arg("expression_parameterization"), py::arg("variables"),
          cls_doc.ctor.doc_2args_expression_parameterization_variables)
      .def(py::init<const multibody::RationalForwardKinematics*,
               const Eigen::Ref<const Eigen::VectorXd>&>(),
          py::arg("kin"), py::arg("q_star_val"),
          cls_doc.ctor.doc_2args_kin_q_star_val)
      .def("get_parameterization_is_threadsafe",
          &IrisParameterizationFunction::get_parameterization_is_threadsafe,
          cls_doc.get_parameterization_is_threadsafe.doc)
      .def("get_parameterization_dimension",
          &IrisParameterizationFunction::get_parameterization_dimension,
          cls_doc.get_parameterization_dimension.doc)
      .def("get_parameterization_double",
          &IrisParameterizationFunction::get_parameterization_double,
          cls_doc.get_parameterization_double.doc)
      .def("get_parameterization_autodiff",
          &IrisParameterizationFunction::get_parameterization_autodiff,
          cls_doc.get_parameterization_autodiff.doc);
}

}  // namespace

namespace internal {

void DefinePlanningIrisCommon(py::module_ m) {
  DefinePlanningCommonSampledIrisOptions(m);
  DefinePlanningIrisParameterizationFunction(m);
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
