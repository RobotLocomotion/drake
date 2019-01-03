#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/systems/systems_pybind.h"
#include "drake/systems/analysis/integrator_base.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"
#include "drake/systems/analysis/runge_kutta3_integrator.h"
#include "drake/systems/analysis/simulator.h"

using std::unique_ptr;

namespace drake {
namespace pydrake {

PYBIND11_MODULE(analysis, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::systems;

  m.doc() = "Bindings for the analysis portion of the Systems framework.";

  py::module::import("pydrake.systems.framework");

  auto bind_scalar_types = [m](auto dummy) {
    constexpr auto& doc = pydrake_doc.drake.systems;
    using T = decltype(dummy);
    DefineTemplateClassWithDefault<IntegratorBase<T>>(
        m, "IntegratorBase", GetPyParam<T>(), doc.IntegratorBase.doc)
        .def("set_fixed_step_mode", &IntegratorBase<T>::set_fixed_step_mode,
            doc.IntegratorBase.set_fixed_step_mode.doc)
        .def("get_fixed_step_mode", &IntegratorBase<T>::get_fixed_step_mode,
            doc.IntegratorBase.get_fixed_step_mode.doc)
        .def("set_target_accuracy", &IntegratorBase<T>::set_target_accuracy,
            doc.IntegratorBase.set_target_accuracy.doc)
        .def("get_target_accuracy", &IntegratorBase<T>::get_target_accuracy,
            doc.IntegratorBase.get_target_accuracy.doc)
        .def("set_maximum_step_size", &IntegratorBase<T>::set_maximum_step_size,
            doc.IntegratorBase.set_maximum_step_size.doc)
        .def("get_maximum_step_size", &IntegratorBase<T>::get_maximum_step_size,
            doc.IntegratorBase.get_maximum_step_size.doc)
        .def("set_requested_minimum_step_size",
            &IntegratorBase<T>::set_requested_minimum_step_size,
            doc.IntegratorBase.set_requested_minimum_step_size.doc)
        .def("get_requested_minimum_step_size",
            &IntegratorBase<T>::get_requested_minimum_step_size,
            doc.IntegratorBase.get_requested_minimum_step_size.doc)
        .def("set_throw_on_minimum_step_size_violation",
            &IntegratorBase<T>::set_throw_on_minimum_step_size_violation,
            doc.IntegratorBase.set_throw_on_minimum_step_size_violation.doc)
        .def("get_throw_on_minimum_step_size_violation",
            &IntegratorBase<T>::get_throw_on_minimum_step_size_violation,
            doc.IntegratorBase.get_throw_on_minimum_step_size_violation.doc);

    DefineTemplateClassWithDefault<RungeKutta2Integrator<T>, IntegratorBase<T>>(
        m, "RungeKutta2Integrator", GetPyParam<T>(),
        doc.RungeKutta2Integrator.doc)
        .def(py::init<const System<T>&, const T&, Context<T>*>(),
            py::arg("system"), py::arg("max_step_size"),
            py::arg("context") = nullptr,
            // Keep alive, reference: `self` keeps `System` alive.
            py::keep_alive<1, 2>(),
            // Keep alive, reference: `self` keeps `Context` alive.
            py::keep_alive<1, 4>(), doc.RungeKutta2Integrator.ctor.doc);

    DefineTemplateClassWithDefault<RungeKutta3Integrator<T>, IntegratorBase<T>>(
        m, "RungeKutta3Integrator", GetPyParam<T>(),
        doc.RungeKutta3Integrator.doc)
        .def(py::init<const System<T>&, Context<T>*>(), py::arg("system"),
            py::arg("context") = nullptr,
            // Keep alive, reference: `self` keeps `System` alive.
            py::keep_alive<1, 2>(),
            // Keep alive, reference: `self` keeps `Context` alive.
            py::keep_alive<1, 3>(), doc.RungeKutta3Integrator.ctor.doc);

    DefineTemplateClassWithDefault<Simulator<T>>(
        m, "Simulator", GetPyParam<T>(), doc.Simulator.doc)
        .def(py::init<const System<T>&, unique_ptr<Context<T>>>(),
            py::arg("system"), py::arg("context") = nullptr,
            // Keep alive, reference: `self` keeps `System` alive.
            py::keep_alive<1, 2>(),
            // Keep alive, ownership: `Context` keeps `self` alive.
            py::keep_alive<3, 1>(), doc.Simulator.ctor.doc)
        .def("Initialize", &Simulator<T>::Initialize,
            doc.Simulator.Initialize.doc)
        .def("StepTo", &Simulator<T>::StepTo, doc.Simulator.StepTo.doc)
        .def("get_context", &Simulator<T>::get_context, py_reference_internal,
            doc.Simulator.get_context.doc)
        .def("get_integrator", &Simulator<T>::get_integrator,
            py_reference_internal, doc.Simulator.get_integrator.doc)
        .def("get_mutable_integrator", &Simulator<T>::get_mutable_integrator,
            py_reference_internal, doc.Simulator.get_mutable_integrator.doc)
        .def("get_mutable_context", &Simulator<T>::get_mutable_context,
            py_reference_internal, doc.Simulator.get_mutable_context.doc)
        .def("reset_integrator",
            [](Simulator<T>* self,
                std::unique_ptr<IntegratorBase<T>> integrator) {
              return self->reset_integrator(std::move(integrator));
            },
            // Keep alive, ownership: 'Integrator' keeps 'self' alive.
            py::keep_alive<2, 1>(),
            doc.Simulator.reset_integrator.doc_1args_stduniqueptr)
        .def("set_publish_every_time_step",
            &Simulator<T>::set_publish_every_time_step,
            doc.Simulator.set_publish_every_time_step.doc)
        .def("set_target_realtime_rate",
            &Simulator<T>::set_target_realtime_rate,
            doc.Simulator.set_target_realtime_rate.doc);
  };
  type_visit(bind_scalar_types, pysystems::NonSymbolicScalarPack{});
}

}  // namespace pydrake
}  // namespace drake
