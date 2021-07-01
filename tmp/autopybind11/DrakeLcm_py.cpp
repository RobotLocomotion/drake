#include "drake/lcm/drake_lcm.h"
#include <pybind11/iostream.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

using namespace drake::lcm;

class DrakeLcm_trampoline : public DrakeLcm {
public:
  typedef DrakeLcm DrakeLcm_alias;
  using DrakeLcm_alias::DrakeLcm;

  int HandleSubscriptions(int arg0) override {
    using localType = int;
    PYBIND11_OVERLOAD(localType, DrakeLcm_alias, HandleSubscriptions, arg0);
  }

  void Publish(std::string const &arg0, void const *arg1, int arg2,
               std::optional<double> arg3) override {
    using localType = void;
    PYBIND11_OVERLOAD(localType, DrakeLcm_alias, Publish, arg0, arg1, arg2,
                      arg3);
  }

  std::shared_ptr<DrakeSubscriptionInterface>
  Subscribe(std::string const &arg0,
            DrakeLcmInterface::HandlerFunction arg1) override {
    using localType = std::shared_ptr<DrakeSubscriptionInterface>;
    PYBIND11_OVERLOAD(localType, DrakeLcm_alias, Subscribe, arg0, arg1);
  }
};

namespace py = pybind11;

void apb11_pydrake_DrakeLcm_py_register(py::module &m) {
  py::class_<DrakeLcm, DrakeLcmInterface, DrakeLcm_trampoline> PyDrakeLcm(
      m, "DrakeLcm");

  PyDrakeLcm.def(py::init<>())
      .def(py::init<::std::string>(), py::arg("lcm_url"))
      .def("HandleSubscriptions",
           static_cast<int (DrakeLcm::*)(int)>(&DrakeLcm::HandleSubscriptions),
           py::arg("timeout_millis"))
      .def("Publish",
           static_cast<void (DrakeLcm::*)(::std::string const &, void const *,
                                          int, ::std::optional<double>)>(
               &DrakeLcm::Publish),
           py::arg("channel"), py::arg("data"), py::arg("data_size"),
           py::arg("time_sec"))
      .def("Subscribe",
           static_cast<::std::shared_ptr<DrakeSubscriptionInterface> (
               DrakeLcm::*)(::std::string const &,
                            DrakeLcmInterface::HandlerFunction)>(
               &DrakeLcm::Subscribe),
           py::arg("channel"), py::arg("arg1"))
      .def("get_lcm_instance", static_cast<::lcm::LCM *(DrakeLcm::*)()>(
                                   &DrakeLcm::get_lcm_instance))
      .def("get_lcm_url", static_cast<::std::string (DrakeLcm::*)() const>(
                              &DrakeLcm::get_lcm_url))

      ;
}
