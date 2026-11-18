#include "drake/bindings/generated_docstrings/multibody_cenic.h"
#include "drake/bindings/pydrake/common/cpp_template_pybind.h"
#include "drake/bindings/pydrake/common/default_scalars_pybind.h"
#include "drake/bindings/pydrake/common/deprecation_pybind.h"
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

    auto cls =
        DefineTemplateClassWithDefault<CenicIntegrator<T>, IntegratorBase<T>>(
            m, "CenicIntegrator", GetPyParam<T>(), doc.CenicIntegrator.doc);
    cls.def(py::init<const System<T>&, Context<T>*>(), py::arg("system"),
           py::arg("context") = nullptr,
           // Keep alive, reference: `self` keeps `system` alive.
           py::keep_alive<1, 2>(),
           // Keep alive, reference: `self` keeps `context` alive.
           py::keep_alive<1, 3>(), doc.CenicIntegrator.ctor.doc)
        .def("GetParameters", &CenicIntegrator<T>::GetParameters,
            doc.CenicIntegrator.GetParameters.doc)
        .def("SetParameters", &CenicIntegrator<T>::SetParameters,
            py::arg("parameters"), doc.CenicIntegrator.SetParameters.doc);
    // Remove on 2027-03-01 per deprecation.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    cls.def("get_solver_parameters",
        WrapDeprecated(doc.CenicIntegrator.get_solver_parameters.doc_deprecated,
            &CenicIntegrator<T>::get_solver_parameters),
        doc.CenicIntegrator.get_solver_parameters.doc_deprecated);
    cls.def("SetSolverParameters",
        WrapDeprecated(doc.CenicIntegrator.SetSolverParameters.doc_deprecated,
            &CenicIntegrator<T>::SetSolverParameters),
        py::arg("parameters"),
        doc.CenicIntegrator.SetSolverParameters.doc_deprecated);
#pragma GCC diagnostic pop
  };
  type_visit(bind_nonsymbolic_scalar_types, NonSymbolicScalarPack{});
}

}  // namespace pydrake
}  // namespace drake
