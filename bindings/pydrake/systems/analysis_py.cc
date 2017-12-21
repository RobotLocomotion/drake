#include <pybind11/pybind11.h>

#include "drake/systems/analysis/simulator.h"

namespace py = pybind11;

using std::unique_ptr;

PYBIND11_MODULE(analysis, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::systems;

  auto py_iref = py::return_value_policy::reference_internal;

  m.doc() = "Bindings for the analysis portion of the Systems framework.";

  using T = double;

  py::class_<Simulator<T>>(m, "Simulator")
    .def(py::init<const System<T>&>())
    .def(py::init<const System<T>&, unique_ptr<Context<T>>>())
    .def("Initialize", &Simulator<T>::Initialize)
    .def("StepTo", &Simulator<T>::StepTo)
    .def("get_context", &Simulator<T>::get_context, py_iref)
    .def("get_mutable_context", &Simulator<T>::get_mutable_context, py_iref);
}
