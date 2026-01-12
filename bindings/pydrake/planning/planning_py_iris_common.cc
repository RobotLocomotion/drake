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

void DefinePlanningCommonSampledIrisOptions(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::planning;
  constexpr auto& doc = pydrake_doc_planning_iris.drake.planning;

  // CommonSampledIrisOptions
  const auto& cls_doc = doc.CommonSampledIrisOptions;
  py::class_<CommonSampledIrisOptions> common_sampled_iris_options(
      m, "CommonSampledIrisOptions", cls_doc.doc);
  common_sampled_iris_options.def(py::init<>())
      .def_readwrite("num_particles", &CommonSampledIrisOptions::num_particles,
          cls_doc.num_particles.doc)
      .def_readwrite("tau", &CommonSampledIrisOptions::tau, cls_doc.tau.doc)
      .def_readwrite(
          "delta", &CommonSampledIrisOptions::delta, cls_doc.delta.doc)
      .def_readwrite(
          "epsilon", &CommonSampledIrisOptions::epsilon, cls_doc.epsilon.doc)
      .def_readwrite("containment_points",
          &CommonSampledIrisOptions::containment_points,
          cls_doc.containment_points.doc)
      .def_readwrite("max_iterations",
          &CommonSampledIrisOptions::max_iterations, cls_doc.max_iterations.doc)
      .def_readwrite("max_iterations_separating_planes",
          &CommonSampledIrisOptions::max_iterations_separating_planes,
          cls_doc.max_iterations_separating_planes.doc)
      .def_readwrite("max_separating_planes_per_iteration",
          &CommonSampledIrisOptions::max_separating_planes_per_iteration,
          cls_doc.max_separating_planes_per_iteration.doc)
      .def_readwrite("parallelism", &CommonSampledIrisOptions::parallelism,
          cls_doc.parallelism.doc)
      .def_readwrite(
          "verbose", &CommonSampledIrisOptions::verbose, cls_doc.verbose.doc)
      .def_readwrite("require_sample_point_is_contained",
          &CommonSampledIrisOptions::require_sample_point_is_contained,
          cls_doc.require_sample_point_is_contained.doc)
      .def_readwrite("configuration_space_margin",
          &CommonSampledIrisOptions::configuration_space_margin,
          cls_doc.configuration_space_margin.doc)
      .def_readwrite("relax_margin", &CommonSampledIrisOptions::relax_margin,
          cls_doc.relax_margin.doc)
      .def_readwrite("termination_threshold",
          &CommonSampledIrisOptions::termination_threshold,
          cls_doc.termination_threshold.doc)
      .def_readwrite("relative_termination_threshold",
          &CommonSampledIrisOptions::relative_termination_threshold,
          cls_doc.relative_termination_threshold.doc)
      .def_readwrite("remove_all_collisions_possible",
          &CommonSampledIrisOptions::remove_all_collisions_possible,
          cls_doc.remove_all_collisions_possible.doc)
      .def_readwrite("random_seed", &CommonSampledIrisOptions::random_seed,
          cls_doc.random_seed.doc)
      .def_readwrite("mixing_steps", &CommonSampledIrisOptions::mixing_steps,
          cls_doc.mixing_steps.doc)
      .def_readwrite("sample_particles_in_parallel",
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

// Largely based on code from solvers_py_mathematicalprogram.cc.
// TODO(cohnt): Refactor for better code reuse.
enum class ArrayShapeType { Scalar, Vector };

// Checks array shape, provides user-friendly message if it fails.
void CheckArrayShape(
    py::str var_name, py::array x, ArrayShapeType shape, int size) {
  bool ndim_is_good{};
  py::str ndim_hint;
  if (shape == ArrayShapeType::Scalar) {
    ndim_is_good = (x.ndim() == 0);
    ndim_hint = "0 (scalar)";
  } else {
    ndim_is_good = (x.ndim() == 1 || x.ndim() == 2);
    ndim_hint = "1 or 2 (vector)";
  }
  if (!ndim_is_good || x.size() != size) {
    throw std::runtime_error(
        py::str("{} must be of .ndim = {} and .size = {}. "
                "Got .ndim = {} and .size = {} instead.")
            .format(var_name, ndim_hint, size, x.ndim(), x.size()));
  }
}

// Checks array type, provides user-friendly message if it fails.
template <typename T>
void CheckReturnedArrayType(py::str cls_name, py::array y) {
  py::module m = py::module::import("pydrake.solvers._extra");
  m.attr("_check_returned_array_type")(cls_name, y, GetPyParam<T>()[0]);
}

// Wraps user function to provide better user-friendliness.
template <typename T, typename Func>
Func WrapParameterizationFunc(
    py::str cls_name, py::function func, int num_vars) {
  py::cpp_function wrapped = [=](py::array x) {
    // Check input.
    // WARNING: If the input is badly sized, we will only reach this error in
    // Release mode. In debug mode, an assertion error will be triggered.
    CheckArrayShape(py::str("{}: Input").format(cls_name), x,
        ArrayShapeType::Vector, num_vars);
    // N.B. We use `py::object` instead of `py::array` for the return type
    /// because for dtype=object, you cannot implicitly cast `np.array(T())`
    // (numpy scalar) to `T` (object), at least for AutoDiffXd.
    py::object y = func(x);
    CheckReturnedArrayType<T>(cls_name, y);
    return y;
  };
  return wrapped.cast<Func>();
}

void DefinePlanningIrisParameterizationFunction(py::module m) {
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

  py::class_<IrisParameterizationFunction> iris_parameterization_function(
      m, "IrisParameterizationFunction", cls_doc.doc);
  iris_parameterization_function.def(py::init<>(), cls_doc.ctor.doc_0args)
      .def(py::init([](const py::function& parameterization,
                        int parameterization_dimension) {
        return IrisParameterizationFunction(
            WrapParameterizationFunc<double,
                IrisParameterizationFunction::ParameterizationFunctionDouble>(
                "IrisParameterizationFunction", parameterization,
                parameterization_dimension),
            WrapParameterizationFunc<AutoDiffXd,
                IrisParameterizationFunction::ParameterizationFunctionAutodiff>(
                "IrisParameterizationFunction", parameterization,
                parameterization_dimension),
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

void DefinePlanningIrisCommon(py::module m) {
  DefinePlanningCommonSampledIrisOptions(m);
  DefinePlanningIrisParameterizationFunction(m);
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
