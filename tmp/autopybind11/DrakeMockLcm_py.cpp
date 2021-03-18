#include "drake/lcm/drake_mock_lcm.h"
#include <pybind11/iostream.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

class DrakeMockLcm_trampoline : public ::drake::lcm::DrakeMockLcm {
public:
  typedef ::drake::lcm::DrakeMockLcm DrakeMockLcm_alias;
  using DrakeMockLcm_alias::DrakeMockLcm;

  int HandleSubscriptions(int arg0) override {
    using localType = int;
    PYBIND11_OVERLOAD(localType, DrakeMockLcm_alias, HandleSubscriptions, arg0);
  }

  void OnHandleSubscriptionsError(std::string const &error_message) override {
    using localType = void;
    PYBIND11_OVERLOAD_PURE(localType, DrakeMockLcm_alias,
                           OnHandleSubscriptionsError, error_message);
  }

  void Publish(std::string const &arg0, void const *arg1, int arg2,
               std::optional<double> arg3) override {
    using localType = void;
    PYBIND11_OVERLOAD(localType, DrakeMockLcm_alias, Publish, arg0, arg1, arg2,
                      arg3);
  }

  std::shared_ptr<drake::lcm::DrakeSubscriptionInterface>
  Subscribe(std::string const &arg0,
            drake::lcm::DrakeLcmInterface::HandlerFunction arg1) override {
    using localType = std::shared_ptr<drake::lcm::DrakeSubscriptionInterface>;
    PYBIND11_OVERLOAD(localType, DrakeMockLcm_alias, Subscribe, arg0, arg1);
  }
};

namespace py = pybind11;
void apb11_pydrake_DrakeMockLcm_py_register(py::module &m) {
  static bool called = false;
  if (called) {
    return;
  }
  called = true;
  using namespace drake::lcm;

  py::class_<DrakeMockLcm, DrakeLcm, DrakeMockLcm_trampoline> PyDrakeMockLcm(
      m, "DrakeMockLcm");

  PyDrakeMockLcm.def(py::init<>())

      ;
}
