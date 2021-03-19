#include "drake/lcm/drake_lcm_interface.h"
#include <pybind11/iostream.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

using namespace drake::lcm;

namespace py = pybind11;
void apb11_pydrake_DrakeLcmInterface_py_register(py::module &m) {
  static bool called = false;
  if (called) {
    return;
  }
  called = true;
  py::class_<DrakeLcmInterface> PyDrakeLcmInterface(m, "DrakeLcmInterface");

  PyDrakeLcmInterface
      .def("HandleSubscriptions",
           static_cast<int (DrakeLcmInterface::*)(int)>(
               &DrakeLcmInterface::HandleSubscriptions),
           py::arg("timeout_millis"))
      .def("Publish",
           static_cast<void (DrakeLcmInterface::*)(
               ::std::string const &, void const *, int,
               ::std::optional<double>)>(&DrakeLcmInterface::Publish),
           py::arg("channel"), py::arg("data"), py::arg("data_size"),
           py::arg("time_sec"))
      .def("Subscribe",
           static_cast<::std::shared_ptr<DrakeSubscriptionInterface> (
               DrakeLcmInterface::*)(::std::string const &,
                                     DrakeLcmInterface::HandlerFunction)>(
               &DrakeLcmInterface::Subscribe),
           py::arg("channel"), py::arg("arg1"))

      ;
}
