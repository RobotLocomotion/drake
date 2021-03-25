#include "drake/lcm/drake_lcm_interface.h"
#include <pybind11/iostream.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

using namespace drake::lcm;

namespace py = pybind11;

void apb11_pydrake_DrakeSubscriptionInterface_py_register(py::module &m) {
  py::class_<DrakeSubscriptionInterface> PyDrakeSubscriptionInterface(
      m, "DrakeSubscriptionInterface");

  PyDrakeSubscriptionInterface
      .def("set_queue_capacity",
           static_cast<void (DrakeSubscriptionInterface::*)(int)>(
               &DrakeSubscriptionInterface::set_queue_capacity),
           py::arg("capacity"))
      .def("set_unsubscribe_on_delete",
           static_cast<void (DrakeSubscriptionInterface::*)(bool)>(
               &DrakeSubscriptionInterface::set_unsubscribe_on_delete),
           py::arg("enabled"))

      ;
}
