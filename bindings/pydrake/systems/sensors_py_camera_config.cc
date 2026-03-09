#include "drake/bindings/generated_docstrings/systems_sensors.h"
#include "drake/bindings/pydrake/common/serialize_pybind.h"
#include "drake/bindings/pydrake/systems/sensors_py.h"
#include "drake/common/overloaded.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/sensors/camera_config_functions.h"

namespace drake {
namespace pydrake {
namespace internal {

using systems::DiagramBuilder;

void DefineSensorsCameraConfig(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::systems::sensors;
  constexpr auto& doc = pydrake_doc_systems_sensors.drake.systems.sensors;

  {
    // To bind nested serializable structs without errors, we declare the outer
    // struct first, then bind its inner structs, then bind the outer struct.
    constexpr auto& config_cls_doc = doc.CameraConfig;
    py::class_<CameraConfig> config_cls(m, "CameraConfig", config_cls_doc.doc);

    // Inner struct.
    constexpr auto& fov_degrees_doc = doc.CameraConfig.FovDegrees;
    py::class_<CameraConfig::FovDegrees> fov_class(
        config_cls, "FovDegrees", fov_degrees_doc.doc);
    fov_class  // BR
        .def(ParamInit<CameraConfig::FovDegrees>())
        .def("focal_x", &CameraConfig::FovDegrees::focal_x, py::arg("width"),
            py::arg("height"), fov_degrees_doc.focal_x.doc)
        .def("focal_y", &CameraConfig::FovDegrees::focal_y, py::arg("width"),
            py::arg("height"), fov_degrees_doc.focal_y.doc);
    DefAttributesUsingSerialize(&fov_class, fov_degrees_doc);
    DefReprUsingSerialize(&fov_class);
    DefCopyAndDeepCopy(&fov_class);

    // Inner struct.
    constexpr auto& focal_doc = doc.CameraConfig.FocalLength;
    py::class_<CameraConfig::FocalLength> focal_class(
        config_cls, "FocalLength", focal_doc.doc);
    focal_class  // BR
        .def(ParamInit<CameraConfig::FocalLength>())
        .def("focal_x", &CameraConfig::FocalLength::focal_x,
            focal_doc.focal_x.doc)
        .def("focal_y", &CameraConfig::FocalLength::focal_y,
            focal_doc.focal_y.doc);
    DefAttributesUsingSerialize(&focal_class, focal_doc);
    DefReprUsingSerialize(&focal_class);
    DefCopyAndDeepCopy(&focal_class);

    // Now we can bind the outer struct (see above).
    config_cls  // BR
        .def(ParamInit<CameraConfig>())
        .def("focal_x", &CameraConfig::focal_x, config_cls_doc.focal_x.doc)
        .def("focal_y", &CameraConfig::focal_y, config_cls_doc.focal_y.doc)
        .def("principal_point", &CameraConfig::principal_point,
            config_cls_doc.principal_point.doc)
        .def("MakeCameras", &CameraConfig::MakeCameras,
            config_cls_doc.MakeCameras.doc);
    DefAttributesUsingSerialize(&config_cls, config_cls_doc);
    DefReprUsingSerialize(&config_cls);
    DefCopyAndDeepCopy(&config_cls);
  }

  m.def(
      "ApplyCameraConfig",
      [](const CameraConfig& config, DiagramBuilder<double>* builder,
          const systems::lcm::LcmBuses* lcm_buses,
          const multibody::MultibodyPlant<double>* plant,
          geometry::SceneGraph<double>* scene_graph,
          std::variant<drake::lcm::DrakeLcmInterface*,
              drake::systems::lcm::LcmInterfaceSystem*>
              lcm) {
        // See kLcmInterfaceSystemClassWarning in pydrake/systems/lcm_py.cc; we
        // need to accept two distinct types in the python signature and upcast
        // inside the C++ implementation here.
        std::visit(
            [&](auto* unboxed_lcm) {
              ApplyCameraConfig(
                  config, builder, lcm_buses, plant, scene_graph, unboxed_lcm);
            },
            lcm);
      },
      py::arg("config"), py::arg("builder"), py::arg("lcm_buses") = nullptr,
      py::arg("plant") = nullptr, py::arg("scene_graph") = nullptr,
      py::arg("lcm") = nullptr,
      // Keep alive, reference: `builder` keeps `lcm` alive.
      py::keep_alive<2, 6>(), doc.ApplyCameraConfig.doc);
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
