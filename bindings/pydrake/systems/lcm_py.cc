#include <cstring>

#include "pybind11/eval.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/systems/systems_pybind.h"
#include "drake/lcm/drake_lcm_interface.h"
#include "drake/systems/lcm/connect_lcm_scope.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/lcm/serializer.h"

namespace drake {
namespace pydrake {

using systems::AbstractValue;
using systems::lcm::SerializerInterface;

namespace {

// pybind11 trampoline class to permit overriding virtual functions in
// Python.
class PySerializerInterface : public py::wrapper<SerializerInterface> {
 public:
  using Base = py::wrapper<SerializerInterface>;

  PySerializerInterface() : Base() {}

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
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::lcm;
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::systems;
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::systems::lcm;
  constexpr auto& doc = pydrake_doc.drake.systems.lcm;

  py::module::import("pydrake.lcm");
  py::module::import("pydrake.systems.framework");

  {
    using Class = SerializerInterface;
    py::class_<Class, PySerializerInterface>(m, "SerializerInterface")
        .def(py::init(
                 []() { return std::make_unique<PySerializerInterface>(); }),
            doc.SerializerInterface.ctor.doc);
    // TODO(eric.cousineau): Consider providing bindings of C++ types if we want
    // to be able to connect to ports which use C++ LCM types.
  }

  {
    using Class = LcmPublisherSystem;
    py::class_<Class, LeafSystem<double>>(m, "LcmPublisherSystem")
        .def(py::init<const std::string&, std::unique_ptr<SerializerInterface>,
                 DrakeLcmInterface*>(),
            py::arg("channel"), py::arg("serializer"), py::arg("lcm"),
            // Keep alive: `self` keeps `DrakeLcmInterface` alive.
            py::keep_alive<1, 3>(), doc.LcmPublisherSystem.ctor.doc)
        .def("set_publish_period", &Class::set_publish_period,
            py::arg("period"), doc.LcmPublisherSystem.set_publish_period.doc);
  }

  {
    using Class = LcmSubscriberSystem;
    py::class_<Class, LeafSystem<double>>(m, "LcmSubscriberSystem")
        .def(py::init<const std::string&, std::unique_ptr<SerializerInterface>,
                 DrakeLcmInterface*>(),
            py::arg("channel"), py::arg("serializer"), py::arg("lcm"),
            // Keep alive: `self` keeps `DrakeLcmInterface` alive.
            py::keep_alive<1, 3>(),
            doc.LcmSubscriberSystem.ctor.doc_3args_channel_serializer_lcm);
  }

  m.def("ConnectLcmScope", &ConnectLcmScope, py::arg("src"), py::arg("channel"),
      py::arg("builder"), py::arg("lcm") = nullptr, py::keep_alive<0, 2>(),
      // TODO(eric.cousineau): Figure out why this is necessary (#9398).
      py_reference, doc.ConnectLcmScope.doc);

  ExecuteExtraPythonCode(m);
}

}  // namespace pydrake
}  // namespace drake
