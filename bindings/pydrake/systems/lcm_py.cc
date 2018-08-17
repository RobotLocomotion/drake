#include <cstring>

#include "pybind11/eval.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/systems/systems_pybind.h"
#include "drake/lcm/drake_lcm_interface.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/lcm/serializer.h"

namespace drake {
namespace pydrake {

using systems::lcm::SerializerInterface;
using systems::AbstractValue;

namespace {

// pybind11 trampoline class to permit overriding virtual functions in
// Python.
class PySerializerInterface : public py::wrapper<SerializerInterface> {
 public:
  using Base = py::wrapper<SerializerInterface>;

  PySerializerInterface()
    : Base() {}

  std::unique_ptr<AbstractValue> CreateDefaultValue() const override {
    PYBIND11_OVERLOAD_PURE(
        std::unique_ptr<AbstractValue>, SerializerInterface,
        CreateDefaultValue);
  }

  void Deserialize(
      const void* message_bytes, int message_length,
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

PYBIND11_MODULE(_lcm_py, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::lcm;
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::systems;
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::systems::lcm;

  py::module::import("pydrake.lcm");
  py::module::import("pydrake.systems.framework");

  {
    using Class = SerializerInterface;
    py::class_<Class, PySerializerInterface>(m, "SerializerInterface")
        .def(py::init([]() {
              return std::make_unique<PySerializerInterface>();
            }));
    // TODO(eric.cousineau): Consider providing bindings of C++ types if we want
    // to be able to connect to ports which use C++ LCM types.
  }

  {
    using Class = LcmPublisherSystem;
    py::class_<Class, LeafSystem<double>>(m, "LcmPublisherSystem")
        .def(
            py::init<const std::string&, std::unique_ptr<SerializerInterface>,
                     DrakeLcmInterface*>(),
            py::arg("channel"), py::arg("serializer"), py::arg("lcm"),
            // Keep alive: `self` keeps `DrakeLcmInterface` alive.
            py::keep_alive<1, 3>())
        .def(
            "set_publish_period", &Class::set_publish_period,
            py::arg("period"));
  }

  {
    using Class = LcmSubscriberSystem;
    py::class_<Class, LeafSystem<double>>(m, "LcmSubscriberSystem")
        .def(
            py::init<const std::string&, std::unique_ptr<SerializerInterface>,
                     DrakeLcmInterface*>(),
            py::arg("channel"), py::arg("serializer"), py::arg("lcm"),
            // Keep alive: `self` keeps `DrakeLcmInterface` alive.
            py::keep_alive<1, 3>());
  }
}

}  // namespace pydrake
}  // namespace drake
