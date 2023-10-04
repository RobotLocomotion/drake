#include <cstring>

#include "pybind11/eval.h"

#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/systems/lcm_py_bind_cpp_serializers.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcm/drake_lcm_interface.h"
#include "drake/systems/lcm/lcm_buses.h"
#include "drake/systems/lcm/lcm_config_functions.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_scope_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/lcm/serializer.h"

namespace drake {
namespace pydrake {

using lcm::DrakeLcm;
using lcm::DrakeLcmInterface;
using lcm::DrakeLcmParams;
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

// Define warning related to lack of multiple inheritance with our fork of
// pybind11. This will be sufficient for workflows in pure Python, but will
// not readily support intermingling of C++-constructed LcmInterfaceSystem
// classes which are then passed to Python.
constexpr char kLcmInterfaceSystemClassWarning[] = R"""(

Warning:
    In C++, this class inherits both LeafSystem and DrakeLcmInterface. However,
    in Python, this only inherits from LeafSystem since our fork of pybind11
    does not support multiple inheritance. Additionally, it only exposes the
    constructor accepting a DrakeLcmInterface, so that it can still be used in
    interface code.
)""";

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
    using Class = LcmInterfaceSystem;
    constexpr auto& cls_doc = doc.LcmInterfaceSystem;
    py::class_<Class, LeafSystem<double>>(m, "LcmInterfaceSystem",
        (std::string(cls_doc.doc) + kLcmInterfaceSystemClassWarning).c_str())
        .def(py::init<DrakeLcmInterface*>(),
            // Keep alive, reference: `self` keeps `lcm` alive.
            py::keep_alive<1, 2>(), py::arg("lcm"), cls_doc.ctor.doc_1args_lcm)
        // Because we can't tell pybind11 that we inherit DrakeLcmInterface,
        // we'll need to manually bind any functions on that interface that we
        // want in Python. For now, we'll just bind the simple ones that don't
        // use function callbacks.
        .def("get_lcm_url", &Class::get_lcm_url,
            pydrake_doc.drake.lcm.DrakeLcmInterface.get_lcm_url.doc)
        .def("HandleSubscriptions", &Class::HandleSubscriptions,
            py::arg("timeout_millis"),
            pydrake_doc.drake.lcm.DrakeLcmInterface.HandleSubscriptions.doc);
  }

  {
    using Class = SerializerInterface;
    constexpr auto& cls_doc = doc.SerializerInterface;
    py::class_<Class, PySerializerInterface, std::shared_ptr<Class>> cls(
        m, "SerializerInterface");
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
        .def(
            "Deserialize",
            [](const Class& self, py::bytes message_bytes,
                AbstractValue* abstract_value) {
              std::string str = message_bytes;
              self.Deserialize(str.data(), str.size(), abstract_value);
            },
            py::arg("message_bytes"), py::arg("abstract_value"),
            cls_doc.Deserialize.doc)
        .def(
            "Serialize",
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
    using Class = LcmBuses;
    constexpr auto& cls_doc = doc.LcmBuses;
    py::class_<Class> cls(m, "LcmBuses");
    cls  // BR
        .def_readonly_static("kLcmUrlMemqNull", &Class::kLcmUrlMemqNull
            // TODO(jwnimmer-tri) The `cls_doc.kLcmUrlMemqNull.doc` docstring
            // constant is absent for some unknown reason, but it wouldn't help
            // anyway because pybind11 throws away docs on static constants:
            // https://github.com/pybind/pybind11/issues/1111
            )
        .def(py::init(), cls_doc.ctor.doc)
        .def("size", &Class::size, cls_doc.size.doc)
        .def("Find", &Class::Find, py::arg("description_of_caller"),
            py::arg("bus_name"), py_rvp::reference_internal, cls_doc.Find.doc)
        .def("GetAllBusNames", &Class::GetAllBusNames,
            cls_doc.GetAllBusNames.doc)
        .def("Add", &Class::Add, py::arg("bus_name"), py::arg("bus"),
            // Keep alive, ownership: `self` keeps `bus` alive.
            py::keep_alive<1, 3>(), cls_doc.Add.doc);
  }

  m.def("ApplyLcmBusConfig",
      py::overload_cast<
          const std::map<std::string, std::optional<DrakeLcmParams>>&,
          systems::DiagramBuilder<double>*>(&ApplyLcmBusConfig),
      py::arg("lcm_buses"), py::arg("builder"), doc.ApplyLcmBusConfig.doc);

  {
    using Class = LcmPublisherSystem;
    constexpr auto& cls_doc = doc.LcmPublisherSystem;
    py::class_<Class, LeafSystem<double>> cls(m, "LcmPublisherSystem");
    cls  // BR
        .def(py::init<const std::string&,
                 std::shared_ptr<const SerializerInterface>,
                 LcmInterfaceSystem*, double, double>(),
            py::arg("channel"), py::arg("serializer"), py::arg("lcm"),
            py::arg("publish_period") = 0.0, py::arg("publish_offset") = 0.0,
            // Keep alive, reference: `self` keeps `lcm` alive.
            py::keep_alive<1, 4>(), cls_doc.ctor.doc_5args)
        .def(py::init<const std::string&,
                 std::shared_ptr<const SerializerInterface>, DrakeLcmInterface*,
                 double, double>(),
            py::arg("channel"), py::arg("serializer"), py::arg("lcm"),
            py::arg("publish_period") = 0.0, py::arg("publish_offset") = 0.0,
            // Keep alive, reference: `self` keeps `lcm` alive.
            py::keep_alive<1, 4>(), cls_doc.ctor.doc_5args)
        .def(
            py::init<const std::string&,
                std::shared_ptr<const SerializerInterface>, LcmInterfaceSystem*,
                const systems::TriggerTypeSet&, double, double>(),
            py::arg("channel"), py::arg("serializer"), py::arg("lcm"),
            py::arg("publish_triggers"), py::arg("publish_period") = 0.0,
            py::arg("publish_offset") = 0.0,
            // Keep alive, reference: `self` keeps `lcm` alive.
            py::keep_alive<1, 4>(), cls_doc.ctor.doc_6args)
        .def(py::init<const std::string&,
                 std::shared_ptr<const SerializerInterface>, DrakeLcmInterface*,
                 const systems::TriggerTypeSet&, double, double>(),
            py::arg("channel"), py::arg("serializer"), py::arg("lcm"),
            py::arg("publish_triggers"), py::arg("publish_period") = 0.0,
            py::arg("publish_offset") = 0.0,
            // Keep alive, reference: `self` keeps `lcm` alive.
            py::keep_alive<1, 4>(), cls_doc.ctor.doc_6args);
  }

  {
    using Class = LcmSubscriberSystem;
    constexpr auto& cls_doc = doc.LcmSubscriberSystem;
    py::class_<Class, LeafSystem<double>>(m, "LcmSubscriberSystem")
        .def(py::init<const std::string&,
                 std::shared_ptr<const SerializerInterface>,
                 LcmInterfaceSystem*, double>(),
            py::arg("channel"), py::arg("serializer"), py::arg("lcm"),
            py::arg("wait_for_message_on_initialization_timeout") = 0.0,
            // Keep alive, reference: `self` keeps `lcm` alive.
            py::keep_alive<1, 4>(), doc.LcmSubscriberSystem.ctor.doc)
        .def(py::init<const std::string&,
                 std::shared_ptr<const SerializerInterface>, DrakeLcmInterface*,
                 double>(),
            py::arg("channel"), py::arg("serializer"), py::arg("lcm"),
            py::arg("wait_for_message_on_initialization_timeout") = 0.0,
            // Keep alive, reference: `self` keeps `lcm` alive.
            py::keep_alive<1, 4>(), doc.LcmSubscriberSystem.ctor.doc)
        .def("WaitForMessage", &Class::WaitForMessage,
            py::arg("old_message_count"), py::arg("message") = nullptr,
            py::arg("timeout") = -1, cls_doc.WaitForMessage.doc);
  }

  {
    using Class = LcmScopeSystem;
    constexpr auto& cls_doc = doc.LcmScopeSystem;
    py::class_<Class, LeafSystem<double>>(m, "LcmScopeSystem")
        .def(py::init<int>(), py::arg("size"), cls_doc.ctor.doc)
        .def_static(
            "AddToBuilder",
            [](systems::DiagramBuilder<double>* builder,
                drake::lcm::DrakeLcmInterface* lcm,
                const OutputPort<double>& signal, const std::string& channel,
                double publish_period) {
              auto [scope, publisher] = LcmScopeSystem::AddToBuilder(
                  builder, lcm, signal, channel, publish_period);
              // Annotate the proper rvp on the tuple of pointers.
              py::object py_builder = py::cast(builder, py_rvp::reference);
              py::list result;
              result.append(
                  py::cast(scope, py_rvp::reference_internal, py_builder));
              result.append(
                  py::cast(publisher, py_rvp::reference_internal, py_builder));
              return result;
            },
            py::arg("builder"), py::arg("lcm"), py::arg("signal"),
            py::arg("channel"), py::arg("publish_period"),
            cls_doc.AddToBuilder.doc);
  }

  // Bind C++ serializers.
  BindCppSerializers();

  ExecuteExtraPythonCode(m);
}

}  // namespace pydrake
}  // namespace drake
