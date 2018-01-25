#include <pybind11/pybind11.h>

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/systems/analysis/simulator.h"

using std::unique_ptr;

namespace drake {
namespace pydrake {

PYBIND11_MODULE(analysis, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::systems;

  m.doc() = "Bindings for the analysis portion of the Systems framework.";

  using T = double;

  py::class_<Simulator<T>>(m, "Simulator")
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
    .def("get_mutable_context", &Simulator<T>::get_mutable_context,
         py_reference_internal)
    .def("set_publish_every_time_step",
         &Simulator<T>::set_publish_every_time_step)
    .def("set_target_realtime_rate", &Simulator<T>::set_target_realtime_rate);
}

}  // namespace pydrake
}  // namespace drake
