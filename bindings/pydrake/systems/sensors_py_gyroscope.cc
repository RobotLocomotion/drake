#include "drake/bindings/generated_docstrings/systems_sensors.h"
#include "drake/bindings/pydrake/common/default_scalars_pybind.h"
#include "drake/bindings/pydrake/systems/sensors_py.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/tree/rigid_body.h"
#include "drake/systems/sensors/gyroscope.h"

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
    using Class = Gyroscope<T>;
    constexpr auto& cls_doc = doc.Gyroscope;
    DefineTemplateClassWithDefault<Gyroscope<T>, systems::LeafSystem<T>>(
        m, "Gyroscope", param, cls_doc.doc)
        .def(py::init<const multibody::RigidBody<T>&,
                 const math::RigidTransform<double>&>(),
            py::arg("body"), py::arg("X_BS"), cls_doc.ctor.doc)
        .def("get_body_poses_input_port", &Class::get_body_poses_input_port,
            py_rvp::reference_internal, cls_doc.get_body_poses_input_port.doc)
        .def("get_body_velocities_input_port",
            &Class::get_body_velocities_input_port, py_rvp::reference_internal,
            cls_doc.get_body_velocities_input_port.doc)
        .def("get_measurement_output_port", &Class::get_measurement_output_port,
            py_rvp::reference_internal, cls_doc.get_measurement_output_port.doc)
        .def("body_index", &Class::body_index, cls_doc.body_index.doc)
        .def("pose", &Class::pose, cls_doc.pose.doc);
    // TODO(#23222) Add static method `AddToDiagram`.
  }
}

}  // namespace

void DefineSensorsGyroscope(py::module m) {
  auto bind_common_scalar_types = [m](auto dummy) {
    using T = decltype(dummy);
    DoScalarDependentDefinitions<T>(m, dummy);
  };

  type_visit(bind_common_scalar_types, CommonScalarPack{});
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
