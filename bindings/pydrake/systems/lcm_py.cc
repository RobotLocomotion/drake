#include <cstring>

#include "pybind11/eval.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/common/deprecation_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/systems/lcm_py_bind_cpp_serializers.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcm/drake_lcm_interface.h"
#include "drake/systems/lcm/connect_lcm_scope.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/lcm/serializer.h"

namespace drake {
namespace pydrake {

using lcm::DrakeLcm;
using lcm::DrakeLcmInterface;
using pysystems::pylcm::BindCppSerializers;
using systems::lcm::SerializerInterface;

namespace {

// pybind11 trampoline class to permit overriding virtual functions in
// Python.
class PySerializerInterface : public py::wrapper<SerializerInterface> {
 public:
  using Base = py::wrapper<SerializerInterface>;

  PySerializerInterface() : Base() {}

  // The following methods are for the pybind11 trampoline class to permit C++
  // to call the correct Python override. This code path is only activated for
  // Python implementations of the class (whose inheritance will pass through
  // `PySerializerInterface`). C++ implementations will use the bindings on the
  // interface below.

  std::unique_ptr<AbstractValue> CreateDefaultValue() const override {
    PYBIND11_OVERLOAD_PURE(std::unique_ptr<AbstractValue>, SerializerInterface,
        CreateDefaultValue);
  }

  void Deserialize(const void* message_bytes, int message_length,
      AbstractValue* abstract_value) const override {
    py::bytes buffer(
        reinterpret_cast<const char*>(message_bytes), message_length);
    PYBIND11_OVERLOAD_PURE(
        void, SerializerInterface, Deserialize, buffer, abstract_value);
  }

  void Serialize(const AbstractValue& abstract_value,
      std::vector<uint8_t>* message_bytes) const override {
    auto wrapped = [&]() -> py::bytes {
      // N.B. We must pass `abstract_value` as a pointer to prevent `pybind11`
      // from copying it.
      PYBIND11_OVERLOAD_PURE(
          py::bytes, SerializerInterface, Serialize, &abstract_value);
    };
    std::string str = wrapped();
    message_bytes->resize(str.size());
    std::copy(str.data(), str.data() + str.size(), message_bytes->data());
  }
};

}  // namespace

PYBIND11_MODULE(lcm, m) {
  PYDRAKE_PREVENT_PYTHON3_MODULE_REIMPORT(m);
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::systems;
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::systems::lcm;
  constexpr auto& doc = pydrake_doc.drake.systems.lcm;

  py::module::import("pydrake.lcm");
  py::module::import("pydrake.systems.framework");

  {
    using Class = SerializerInterface;
    constexpr auto& cls_doc = doc.SerializerInterface;
    py::class_<Class, PySerializerInterface> cls(m, "SerializerInterface");
    cls  // BR
         // Adding a constructor permits implementing this interface in Python.
        .def(py::init(
                 []() { return std::make_unique<PySerializerInterface>(); }),
            cls_doc.ctor.doc);
    // The following bindings are present to allow Python to call C++
    // implementations of this interface. Python implementations of the
    // interface will call the trampoline implementation methods above.
    cls  // BR
        .def("CreateDefaultValue", &Class::CreateDefaultValue,
            cls_doc.CreateDefaultValue.doc)
        .def("Deserialize",
            [](const Class& self, py::bytes message_bytes,
                AbstractValue* abstract_value) {
              std::string str = message_bytes;
              self.Deserialize(str.data(), str.size(), abstract_value);
            },
            py::arg("message_bytes"), py::arg("abstract_value"),
            cls_doc.Deserialize.doc)
        .def("Serialize",
            [](const Class& self, const AbstractValue& abstract_value) {
              std::vector<uint8_t> message_bytes;
              self.Serialize(abstract_value, &message_bytes);
              return py::bytes(
                  reinterpret_cast<const char*>(message_bytes.data()),
                  message_bytes.size());
            },
            py::arg("abstract_value"), cls_doc.Serialize.doc);
  }

  {
    using Class = LcmPublisherSystem;
    constexpr auto& cls_doc = doc.LcmPublisherSystem;
    py::class_<Class, LeafSystem<double>> cls(m, "LcmPublisherSystem");
    cls  // BR
        .def(py::init<const std::string&, std::unique_ptr<SerializerInterface>,
                 DrakeLcmInterface*, double>(),
            py::arg("channel"), py::arg("serializer"), py::arg("lcm"),
            py::arg("publish_period") = 0.0,
            // Keep alive, ownership: `serializer` keeps `self` alive.
            py::keep_alive<3, 1>(),
            // Keep alive, reference: `self` keeps `lcm` alive.
            py::keep_alive<1, 4>(), cls_doc.ctor.doc_4args);
  }

  {
    using Class = LcmSubscriberSystem;
    constexpr auto& cls_doc = doc.LcmSubscriberSystem;
    py::class_<Class, LeafSystem<double>>(m, "LcmSubscriberSystem")
        .def(py::init<const std::string&, std::unique_ptr<SerializerInterface>,
                 DrakeLcmInterface*>(),
            py::arg("channel"), py::arg("serializer"), py::arg("lcm"),
            // Keep alive, ownership: `serializer` keeps `self` alive.
            py::keep_alive<3, 1>(),
            // Keep alive, reference: `self` keeps `lcm` alive.
            py::keep_alive<1, 4>(), doc.LcmSubscriberSystem.ctor.doc)
        .def("WaitForMessage", &Class::WaitForMessage,
            py::arg("old_message_count"), py::arg("message") = nullptr,
            py::arg("timeout") = -1, cls_doc.WaitForMessage.doc);
  }

  m.def("ConnectLcmScope", &ConnectLcmScope, py::arg("src"), py::arg("channel"),
      py::arg("builder"), py::arg("lcm") = nullptr, py::keep_alive<0, 2>(),
      // See #11531 for why `py_reference` is needed.
      py_reference, doc.ConnectLcmScope.doc);

  // Bind C++ serializers.
  BindCppSerializers();

  ExecuteExtraPythonCode(m);
}

}  // namespace pydrake
}  // namespace drake
