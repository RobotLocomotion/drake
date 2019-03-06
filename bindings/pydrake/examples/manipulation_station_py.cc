#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "drake/bindings/pydrake/common/deprecation_pybind.h"
#include "drake/bindings/pydrake/common/drake_optional_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/examples/manipulation_station/manipulation_station.h"
#include "drake/examples/manipulation_station/manipulation_station_hardware_interface.h"  // noqa

using std::make_unique;
using std::unique_ptr;
using std::vector;

namespace drake {
namespace pydrake {

PYBIND11_MODULE(manipulation_station, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::systems;
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::examples::manipulation_station;

  m.doc() = "Bindings for the Manipulation Station example.";
  constexpr auto& doc = pydrake_doc.drake.examples.manipulation_station;

  py::module::import("pydrake.systems.framework");

  // ManipulationStation currently only supports double.
  using T = double;

  py::enum_<IiwaCollisionModel>(m, "IiwaCollisionModel")
      .value("kNoCollision", IiwaCollisionModel::kNoCollision,
          doc.IiwaCollisionModel.kNoCollision.doc)
      .value("kBoxCollision", IiwaCollisionModel::kBoxCollision,
          doc.IiwaCollisionModel.kBoxCollision.doc)
      .export_values();

  // TODO(siyuan.feng): Add RegisterRgbdCamera when we have bindings for
  // creating a geometry::dev::render::DepthCameraProperties struct.
  py::class_<ManipulationStation<T>, Diagram<T>>(m, "ManipulationStation")
      .def(py::init<double>(), py::arg("time_step") = 0.002,
          doc.ManipulationStation.ctor.doc)
      .def("SetupDefaultStation", &ManipulationStation<T>::SetupDefaultStation,
          py::arg("collision_model") = IiwaCollisionModel::kNoCollision,
          doc.ManipulationStation.SetupDefaultStation.doc)
      .def("SetupClutterClearingStation",
          &ManipulationStation<T>::SetupClutterClearingStation,
          py::arg("X_WCameraBody") = nullopt,
          py::arg("collision_model") = IiwaCollisionModel::kNoCollision,
          doc.ManipulationStation.SetupClutterClearingStation.doc)
      .def("AddManipulandFromFile",
          &ManipulationStation<T>::AddManipulandFromFile, py::arg("model_file"),
          py::arg("X_WObject"),
          doc.ManipulationStation.AddManipulandFromFile.doc)
      .def("RegisterIiwaControllerModel",
          &ManipulationStation<T>::RegisterIiwaControllerModel,
          doc.ManipulationStation.RegisterIiwaControllerModel.doc)
      .def("RegisterWsgControllerModel",
          &ManipulationStation<T>::RegisterWsgControllerModel,
          doc.ManipulationStation.RegisterWsgControllerModel.doc)
      .def("Finalize", py::overload_cast<>(&ManipulationStation<T>::Finalize),
          doc.ManipulationStation.Finalize.doc_0args)
      .def("get_multibody_plant", &ManipulationStation<T>::get_multibody_plant,
          py_reference_internal,
          doc.ManipulationStation.get_multibody_plant.doc)
      .def("get_mutable_multibody_plant",
          &ManipulationStation<T>::get_mutable_multibody_plant,
          py_reference_internal,
          doc.ManipulationStation.get_mutable_multibody_plant.doc)
      .def("get_scene_graph", &ManipulationStation<T>::get_scene_graph,
          py_reference_internal, doc.ManipulationStation.get_scene_graph.doc)
      .def("get_mutable_scene_graph",
          &ManipulationStation<T>::get_mutable_scene_graph,
          py_reference_internal,
          doc.ManipulationStation.get_mutable_scene_graph.doc)
      .def("get_controller_plant",
          &ManipulationStation<T>::get_controller_plant, py_reference_internal,
          doc.ManipulationStation.get_controller_plant.doc)
      .def("GetIiwaPosition", &ManipulationStation<T>::GetIiwaPosition,
          doc.ManipulationStation.GetIiwaPosition.doc)
      .def("SetIiwaPosition",
          overload_cast_explicit<void, systems::Context<T>*,
              const Eigen::Ref<const VectorX<T>>&>(
              &ManipulationStation<T>::SetIiwaPosition),
          py::arg("station_context"), py::arg("q"),
          doc.ManipulationStation.SetIiwaPosition.doc_2args)
      .def("SetIiwaPosition",
          [](ManipulationStation<T>* self,
              const Eigen::Ref<const VectorX<T>>& q,
              systems::Context<T>* context) {
            WarnDeprecated(
                "SetIiwaPosition(q, context) is deprecated.  Please use "
                "(context, q) instead.");
            self->SetIiwaPosition(context, q);
          })
      .def("GetIiwaVelocity", &ManipulationStation<T>::GetIiwaVelocity,
          doc.ManipulationStation.GetIiwaVelocity.doc)
      .def("SetIiwaVelocity",
          overload_cast_explicit<void, systems::Context<T>*,
              const Eigen::Ref<const VectorX<T>>&>(
              &ManipulationStation<T>::SetIiwaVelocity),
          py::arg("station_context"), py::arg("v"),
          doc.ManipulationStation.SetIiwaVelocity.doc_2args)
      .def("SetIiwaVelocity",
          [](ManipulationStation<T>* self,
              const Eigen::Ref<const VectorX<T>>& v,
              systems::Context<T>* context) {
            WarnDeprecated(
                "SetIiwaVelocity(v, context) is deprecated.  Please use "
                "(context, v) instead.");
            self->SetIiwaVelocity(context, v);
          })
      .def("GetWsgPosition", &ManipulationStation<T>::GetWsgPosition,
          doc.ManipulationStation.GetWsgPosition.doc)
      .def("SetWsgPosition",
          overload_cast_explicit<void, systems::Context<T>*, const T&>(
              &ManipulationStation<T>::SetWsgPosition),
          py::arg("station_context"), py::arg("q"),
          doc.ManipulationStation.SetWsgPosition.doc_2args)
      .def("SetWsgPosition",
          [](ManipulationStation<T>* self, const T& q,
              systems::Context<T>* context) {
            WarnDeprecated(
                "SetWsgPosition(q, context) is deprecated.  Please use "
                "(context, q) instead.");
            self->SetWsgPosition(context, q);
          })
      .def("GetWsgVelocity", &ManipulationStation<T>::GetWsgVelocity,
          doc.ManipulationStation.GetWsgVelocity.doc)
      .def("SetWsgVelocity",
          overload_cast_explicit<void, systems::Context<T>*, const T&>(
              &ManipulationStation<T>::SetWsgVelocity),
          py::arg("station_context"), py::arg("v"),
          doc.ManipulationStation.SetWsgVelocity.doc_2args)
      .def("SetWsgVelocity",
          [](ManipulationStation<T>* self, const T& v,
              systems::Context<T>* context) {
            WarnDeprecated(
                "SetWsgVelocity(v, context) is deprecated.  Please use "
                "(context, v) instead.");
            self->SetWsgVelocity(context, v);
          })
      .def("GetStaticCameraPosesInWorld",
          &ManipulationStation<T>::GetStaticCameraPosesInWorld,
          py_reference_internal,
          doc.ManipulationStation.GetStaticCameraPosesInWorld.doc)
      .def("get_camera_names", &ManipulationStation<T>::get_camera_names,
          doc.ManipulationStation.get_camera_names.doc)
      .def("SetWsgGains", &ManipulationStation<T>::SetWsgGains,
          doc.ManipulationStation.SetWsgGains.doc)
      .def("SetIiwaPositionGains",
          &ManipulationStation<T>::SetIiwaPositionGains,
          doc.ManipulationStation.SetIiwaPositionGains.doc)
      .def("SetIiwaVelocityGains",
          &ManipulationStation<T>::SetIiwaVelocityGains,
          doc.ManipulationStation.SetIiwaVelocityGains.doc)
      .def("SetIiwaIntegralGains",
          &ManipulationStation<T>::SetIiwaIntegralGains,
          doc.ManipulationStation.SetIiwaIntegralGains.doc);

  py::class_<ManipulationStationHardwareInterface, Diagram<double>>(
      m, "ManipulationStationHardwareInterface")
      .def(py::init<const std::vector<std::string>>(),
          py::arg("camera_names") = std::vector<std::string>{},
          doc.ManipulationStationHardwareInterface.ctor.doc)
      .def("Connect", &ManipulationStationHardwareInterface::Connect,
          py::arg("wait_for_cameras") = true,
          doc.ManipulationStationHardwareInterface.Connect.doc)
      .def("get_controller_plant",
          &ManipulationStationHardwareInterface::get_controller_plant,
          py_reference_internal,
          doc.ManipulationStationHardwareInterface.get_controller_plant.doc)
      .def("get_camera_names",
          &ManipulationStationHardwareInterface::get_camera_names,
          py_reference_internal,
          doc.ManipulationStationHardwareInterface.get_camera_names.doc);

  ExecuteExtraPythonCode(m);
}

}  // namespace pydrake
}  // namespace drake
