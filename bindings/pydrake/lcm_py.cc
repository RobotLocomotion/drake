#include <cstring>

#include "pybind11/functional.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "drake/bindings/pydrake/common/deprecation_pybind.h"
#include "drake/bindings/pydrake/common/drake_optional_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcm/drake_lcm_interface.h"
#include "drake/lcm/drake_mock_lcm.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(lcm, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::lcm;
  constexpr auto& doc = pydrake_doc.drake.lcm;

  // Use `py::bytes` as a mid-point between C++ LCM (`void* + int` /
  // `vector<uint8_t>`) and Python LCM (`str`).
  using PyHandlerFunction = std::function<void(py::bytes)>;

  {
    using Class = DrakeLcmInterface;
    constexpr auto& cls_doc = doc.DrakeLcmInterface;
    py::class_<Class>(m, "DrakeLcmInterface", cls_doc.doc)
        // N.B. We do not bind `Subscribe` as multi-threading from C++ may
        // wreak havoc on the Python GIL with a callback.
        .def("Publish",
            [](Class* self, const std::string& channel, py::bytes buffer,
                optional<double> time_sec) {
              // TODO(eric.cousineau): See if there is a way to extra the raw
              // bytes from `buffer` without copying.
              std::string str = buffer;
              self->Publish(channel, str.data(), str.size(), time_sec);
            },
            py::arg("channel"), py::arg("buffer"),
            py::arg("time_sec") = py::none(), cls_doc.Publish.doc)
        .def("HandleSubscriptions", &DrakeLcmInterface::HandleSubscriptions,
            py::arg("timeout_millis"), cls_doc.HandleSubscriptions.doc);
  }

  {
    using Class = DrakeLcm;
    constexpr auto& cls_doc = doc.DrakeLcm;
    py::class_<Class, DrakeLcmInterface>(m, "DrakeLcm", cls_doc.doc)
        .def(py::init<>(), cls_doc.ctor.doc_0args)
        .def(py::init<std::string>(), py::arg("lcm_url"),
            cls_doc.ctor.doc_1args);
    // TODO(eric.cousineau): Add remaining methods.
  }

  {
    using Class = DrakeMockLcm;
    constexpr auto& cls_doc = doc.DrakeMockLcm;
    py::class_<Class, DrakeLcmInterface>(m, "DrakeMockLcm", cls_doc.doc)
        .def(py::init<>(), cls_doc.ctor.doc)
        .def("Subscribe",
            [](Class* self, const std::string& channel,
                PyHandlerFunction handler) {
              auto subscription = self->Subscribe(
                  channel, [handler](const void* data, int size) {
                    handler(py::bytes(static_cast<const char*>(data), size));
                  });
              // Unsubscribe is not supported by the mock.
              DRAKE_DEMAND(subscription == nullptr);
            },
            py::arg("channel"), py::arg("handler"), cls_doc.Subscribe.doc);
  }

  ExecuteExtraPythonCode(m);
}

}  // namespace pydrake
}  // namespace drake
