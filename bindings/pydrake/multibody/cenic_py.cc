#include "drake/bindings/generated_docstrings/multibody_cenic.h"
#include "drake/bindings/pydrake/common/cpp_template_pybind.h"
#include "drake/bindings/pydrake/common/default_scalars_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/multibody/cenic/cenic_integrator.h"

namespace drake {
namespace pydrake {

using multibody::contact_solvers::icf::IcfSolverParameters;
using systems::Context;
using systems::IntegratorBase;
using systems::System;

PYBIND11_MODULE(cenic, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::multibody;
  constexpr auto& doc = pydrake_doc_multibody_cenic.drake.multibody;

  // TODO(jwnimmer-tri) Figure out the right imports.
  py::module::import("pydrake.systems.analysis");

  // ICF Solver Parameters
  // TODO(CENIC): wire up the docstrings properly
  {
    py::class_<IcfSolverParameters>(m, "IcfSolverParameters")
        .def(py::init<>())
        .def_readwrite("max_iterations", &IcfSolverParameters::max_iterations)
        .def_readwrite("min_tolerance", &IcfSolverParameters::min_tolerance)
        .def_readwrite(
            "enable_hessian_reuse", &IcfSolverParameters::enable_hessian_reuse)
        .def_readwrite("hessian_reuse_target_iterations",
            &IcfSolverParameters::hessian_reuse_target_iterations)
        .def_readwrite(
            "use_dense_algebra", &IcfSolverParameters::use_dense_algebra)
        .def_readwrite("max_linesearch_iterations",
            &IcfSolverParameters::max_linesearch_iterations)
        .def_readwrite(
            "linesearch_tolerance", &IcfSolverParameters::linesearch_tolerance)
        .def_readwrite("alpha_max", &IcfSolverParameters::alpha_max)
        .def_readwrite(
            "print_solver_stats", &IcfSolverParameters::print_solver_stats);
  }

  auto bind_nonsymbolic_scalar_types = [&m](auto dummy) {
    using T = decltype(dummy);

    // TODO(vincekurtz): add bindings set IcfSolverParameters
    DefineTemplateClassWithDefault<CenicIntegrator<T>, IntegratorBase<T>>(
        m, "CenicIntegrator", GetPyParam<T>(), doc.CenicIntegrator.doc)
        .def(py::init<const System<T>&, Context<T>*>(), py::arg("system"),
            py::arg("context") = nullptr,
            // Keep alive, reference: `self` keeps `system` alive.
            py::keep_alive<1, 2>(),
            // Keep alive, reference: `self` keeps `context` alive.
            py::keep_alive<1, 3>(), doc.CenicIntegrator.ctor.doc)
        .def("get_solver_parameters",
            &CenicIntegrator<T>::get_solver_parameters,
            doc.CenicIntegrator.get_solver_parameters.doc)
        .def("set_solver_parameters",
            &CenicIntegrator<T>::set_solver_parameters, py::arg("parameters"),
            doc.CenicIntegrator.set_solver_parameters.doc);
  };
  type_visit(bind_nonsymbolic_scalar_types, NonSymbolicScalarPack{});
}

}  // namespace pydrake
}  // namespace drake
