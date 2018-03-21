#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcm/drake_lcm_interface.h"
#include "drake/lcm/drake_mock_lcm.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(lcm, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::lcm;

  {
    using Class = DrakeLcmInterface;
    py::class_<Class>(m, "DrakeLcmInterface");
    // TODO(eric.cousineau): Add remaining methods.
    // TODO(eric.cousineau): Allow virtual overrides in Python.
  }

  {
    using Class = DrakeLcm;
    py::class_<Class, DrakeLcmInterface>(m, "DrakeLcm")
        .def(py::init<>())
        .def("StartReceiveThread", &Class::StartReceiveThread)
        .def("StopReceiveThread", &Class::StopReceiveThread);
    // TODO(eric.cousineau): Add remaining methods.
  }

  {
    using Class = DrakeMockLcm;
    py::class_<Class, DrakeLcmInterface>(m, "DrakeMockLcm")
        .def(py::init<>());
    // TODO(eric.cousineau): Add remaining methods.
  }
}

}  // namespace pydrake
}  // namespace drake
