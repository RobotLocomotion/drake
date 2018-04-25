#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/systems/systems_pybind.h"
#include "drake/systems/analysis/integrator_base.h"
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
    using T = decltype(dummy);
    DefineTemplateClassWithDefault<IntegratorBase<T>>(
        m, "IntegratorBase", GetPyParam<T>())
      .def("set_fixed_step_mode", &IntegratorBase<T>::set_fixed_step_mode)
      .def("get_fixed_step_mode", &IntegratorBase<T>::get_fixed_step_mode)
      .def("set_target_accuracy", &IntegratorBase<T>::set_target_accuracy)
      .def("get_target_accuracy", &IntegratorBase<T>::get_target_accuracy)
      .def("set_maximum_step_size", &IntegratorBase<T>::set_maximum_step_size)
      .def("get_maximum_step_size", &IntegratorBase<T>::get_maximum_step_size)
      .def("set_requested_minimum_step_size",
           &IntegratorBase<T>::set_requested_minimum_step_size)
      .def("get_requested_minimum_step_size",
           &IntegratorBase<T>::get_requested_minimum_step_size)
      .def("set_throw_on_minimum_step_size_violation",
           &IntegratorBase<T>::set_throw_on_minimum_step_size_violation)
      .def("get_throw_on_minimum_step_size_violation",
           &IntegratorBase<T>::get_throw_on_minimum_step_size_violation);

    DefineTemplateClassWithDefault<Simulator<T>>(
        m, "Simulator", GetPyParam<T>())
      .def(py::init<const System<T>&>(),
           // Keep alive, reference: `self` keeps `System` alive.
           py::keep_alive<1, 2>())
      .def(py::init<const System<T>&, unique_ptr<Context<T>>>(),
           // Keep alive, reference: `self` keeps `System` alive.
           py::keep_alive<1, 2>(),
           // Keep alive, ownership: `Context` keeps `self` alive.
           py::keep_alive<3, 1>())
      .def("Initialize", &Simulator<T>::Initialize)
      .def("StepTo", &Simulator<T>::StepTo)
      .def("get_context", &Simulator<T>::get_context, py_reference_internal)
      .def("get_integrator", &Simulator<T>::get_integrator,
           py_reference_internal)
      .def("get_mutable_integrator", &Simulator<T>::get_mutable_integrator,
           py_reference_internal)
      .def("get_mutable_context", &Simulator<T>::get_mutable_context,
           py_reference_internal)
      .def("set_publish_every_time_step",
           &Simulator<T>::set_publish_every_time_step)
      .def("set_target_realtime_rate", &Simulator<T>::set_target_realtime_rate);
  };
  type_visit(bind_scalar_types, pysystems::NonSymbolicScalarPack{});
}

}  // namespace pydrake
}  // namespace drake
