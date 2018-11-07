#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

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

  py::class_<ManipulationStation<T>, Diagram<T>> stat(m,
      "ManipulationStation");

  py::enum_<ManipulationStation<T>::IiwaCollisionModel>(
      stat, "IiwaCollisionModel")
      .value("kNoCollision",
             ManipulationStation<T>::IiwaCollisionModel::kNoCollision,
             doc.ManipulationStation.IiwaCollisionModel.kNoCollision.doc)
      .value("kBoxCollision",
             ManipulationStation<T>::IiwaCollisionModel::kBoxCollision,
             doc.ManipulationStation.IiwaCollisionModel.kBoxCollision.doc)
      .export_values();

  stat.def(py::init<double, ManipulationStation<T>::IiwaCollisionModel>(),
           py::arg("time_step") = 0.002, py::arg("collision_model") =
             ManipulationStation<T>::IiwaCollisionModel::kNoCollision,
           doc.ManipulationStation.ctor.doc_3);
  stat.def("AddCupboard", &ManipulationStation<T>::AddCupboard,
           doc.ManipulationStation.AddCupboard.doc);
  stat.def("Finalize", &ManipulationStation<T>::Finalize,
           doc.ManipulationStation.Finalize.doc);
  stat.def("get_mutable_multibody_plant",
           &ManipulationStation<T>::get_mutable_multibody_plant,
           py_reference_internal,
           doc.ManipulationStation.get_mutable_multibody_plant.doc);
  stat.def("get_mutable_scene_graph",
           &ManipulationStation<T>::get_mutable_scene_graph,
           py_reference_internal,
           doc.ManipulationStation.get_mutable_scene_graph.doc);
  stat.def("get_controller_plant",
           &ManipulationStation<T>::get_controller_plant, py_reference_internal,
           doc.ManipulationStation.get_controller_plant.doc);
  stat.def("GetIiwaPosition", &ManipulationStation<T>::GetIiwaPosition,
           doc.ManipulationStation.GetIiwaPosition.doc);
  stat.def("SetIiwaPosition", &ManipulationStation<T>::SetIiwaPosition,
           doc.ManipulationStation.SetIiwaPosition.doc);
  stat.def("GetIiwaVelocity", &ManipulationStation<T>::GetIiwaVelocity,
           doc.ManipulationStation.GetIiwaVelocity.doc);
  stat.def("SetIiwaVelocity", &ManipulationStation<T>::SetIiwaVelocity,
           doc.ManipulationStation.SetIiwaVelocity.doc);
  stat.def("GetWsgPosition", &ManipulationStation<T>::GetWsgPosition,
           doc.ManipulationStation.GetWsgPosition.doc);
  stat.def("SetWsgPosition", &ManipulationStation<T>::SetWsgPosition,
           doc.ManipulationStation.SetWsgPosition.doc);
  stat.def("GetWsgVelocity", &ManipulationStation<T>::GetWsgVelocity,
           doc.ManipulationStation.GetWsgVelocity.doc);
  stat.def("SetWsgVelocity", &ManipulationStation<T>::SetWsgVelocity,
           doc.ManipulationStation.SetWsgVelocity.doc);
  stat.def_static("get_camera_pose", &ManipulationStation<T>::get_camera_pose,
                  doc.ManipulationStation.get_camera_pose.doc);

  py::class_<ManipulationStationHardwareInterface, Diagram<double>>(
      m, "ManipulationStationHardwareInterface")
      .def(py::init<const std::vector<std::string>>(),
           py::arg("camera_ids") = std::vector<std::string>{},
           doc.ManipulationStationHardwareInterface.ctor.doc_3)
      .def("Connect", &ManipulationStationHardwareInterface::Connect,
           py::arg("wait_for_cameras") = true,
           doc.ManipulationStationHardwareInterface.Connect.doc)
      .def("get_controller_plant",
           &ManipulationStationHardwareInterface::get_controller_plant,
           py_reference_internal,
           doc.ManipulationStationHardwareInterface.get_controller_plant.doc);
}

}  // namespace pydrake
}  // namespace drake
