#include <vector>

#include "drake/bindings/generated_docstrings/systems_sensors.h"
#include "drake/bindings/pydrake/common/default_scalars_pybind.h"
#include "drake/bindings/pydrake/systems/sensors_py.h"
#include "drake/systems/sensors/rotary_encoders.h"

namespace drake {
namespace pydrake {
namespace internal {
namespace {

template <typename T>
void DoScalarDependentDefinitions(py::module m, T) {
  py::tuple param = GetPyParam<T>();

  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::systems::sensors;
  constexpr auto& doc = pydrake_doc_systems_sensors.drake.systems.sensors;

  {
    using Class = RotaryEncoders<T>;
    constexpr auto& cls_doc = doc.RotaryEncoders;
    DefineTemplateClassWithDefault<RotaryEncoders<T>, systems::VectorSystem<T>>(
        m, "RotaryEncoders", param, doc.RotaryEncoders.doc)
        .def(py::init<const std::vector<int>&>(),
            py::arg("ticks_per_revolution"), cls_doc.ctor.doc_1args)
        .def(py::init<int, const std::vector<int>&>(),
            py::arg("input_port_size"), py::arg("input_vector_indices"),
            cls_doc.ctor.doc_2args)
        .def(py::init<int, const std::vector<int>&, const std::vector<int>&>(),
            py::arg("input_port_size"), py::arg("input_vector_indices"),
            py::arg("ticks_per_revolution"), cls_doc.ctor.doc_3args)
        .def("set_calibration_offsets", &Class::set_calibration_offsets,
            py::arg("context"), py::arg("calibration_offsets"),
            cls_doc.set_calibration_offsets.doc)
        .def("get_calibration_offsets", &Class::get_calibration_offsets,
            py::arg("context"), py_rvp::copy,
            cls_doc.get_calibration_offsets.doc);
  }
}

}  // namespace

void DefineSensorsRotaryEncoders(py::module m) {
  type_visit([m](auto dummy) { DoScalarDependentDefinitions(m, dummy); },
      CommonScalarPack{});
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
