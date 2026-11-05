#include "drake/bindings/generated_docstrings/multibody_cenic.h"
#include "drake/bindings/pydrake/common/cpp_template_pybind.h"
#include "drake/bindings/pydrake/common/default_scalars_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/multibody/cenic/cenic_integrator.h"

namespace drake {
namespace pydrake {

using systems::Context;
using systems::IntegratorBase;
using systems::System;

PYDRAKE_MODULE(cenic, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::multibody;
  constexpr auto& doc = pydrake_doc_multibody_cenic.drake.multibody;

  py::module_::import_("pydrake.multibody.contact_solvers");
  py::module_::import_("pydrake.multibody.plant");
  py::module_::import_("pydrake.systems.analysis");

  // CenicIntegratorParameters is bound (privately) by the analysis module,
  // because SimulatorConfig refers to it. Re-export it here, where it belongs.
  // (The analysis module already gave the class its public name and module.)
  py::setattr(m, "CenicIntegratorParameters",
      py::module_::import_("pydrake.systems.analysis")
          .attr("_CenicIntegratorParameters"));

  auto bind_nonsymbolic_scalar_types = [&m](auto dummy) {
    using T = decltype(dummy);

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
        .def("SetSolverParameters", &CenicIntegrator<T>::SetSolverParameters,
            py::arg("parameters"), doc.CenicIntegrator.SetSolverParameters.doc)
        .def("get_explicit_external_inputs",
            &CenicIntegrator<T>::get_explicit_external_inputs,
            doc.CenicIntegrator.get_explicit_external_inputs.doc)
        .def("set_explicit_external_inputs",
            &CenicIntegrator<T>::set_explicit_external_inputs,
            py::arg("explicit_external_inputs"),
            doc.CenicIntegrator.set_explicit_external_inputs.doc);
  };
  type_visit(bind_nonsymbolic_scalar_types, NonSymbolicScalarPack{});
}

}  // namespace pydrake
}  // namespace drake
