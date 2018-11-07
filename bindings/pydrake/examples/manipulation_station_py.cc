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

  py::class_<ManipulationStation<T>, Diagram<T>> station(m,
                                                         "ManipulationStation");

  py::enum_<ManipulationStation<T>::IiwaCollisionModel>(station,
                                                        "IiwaCollisionModel")
      .value("kNoCollision",
             ManipulationStation<T>::IiwaCollisionModel::kNoCollision,
             doc.ManipulationStation.IiwaCollisionModel.kNoCollision.doc)
      .value("kBoxCollision",
             ManipulationStation<T>::IiwaCollisionModel::kBoxCollision,
             doc.ManipulationStation.IiwaCollisionModel.kBoxCollision.doc)
      .export_values();

  station.def(py::init<double, ManipulationStation<T>::IiwaCollisionModel>(),
              py::arg("time_step") = 0.002,
              py::arg("collision_model") =
                  ManipulationStation<T>::IiwaCollisionModel::kNoCollision,
              doc.ManipulationStation.ctor.doc_3);
  station.def("AddCupboard", &ManipulationStation<T>::AddCupboard,
              doc.ManipulationStation.AddCupboard.doc);
  station.def("Finalize", &ManipulationStation<T>::Finalize,
              doc.ManipulationStation.Finalize.doc);
  station.def(
      "get_multibody_plant", &ManipulationStation<T>::get_multibody_plant,
      py_reference_internal, doc.ManipulationStation.get_multibody_plant.doc);
  station.def("get_mutable_multibody_plant",
              &ManipulationStation<T>::get_mutable_multibody_plant,
              py_reference_internal,
              doc.ManipulationStation.get_mutable_multibody_plant.doc);
  station.def("get_scene_graph", &ManipulationStation<T>::get_scene_graph,
              py_reference_internal,
              doc.ManipulationStation.get_scene_graph.doc);
  station.def("get_mutable_scene_graph",
              &ManipulationStation<T>::get_mutable_scene_graph,
              py_reference_internal,
              doc.ManipulationStation.get_mutable_scene_graph.doc);
  station.def(
      "get_controller_plant", &ManipulationStation<T>::get_controller_plant,
      py_reference_internal, doc.ManipulationStation.get_controller_plant.doc);
  station.def("GetIiwaPosition", &ManipulationStation<T>::GetIiwaPosition,
              doc.ManipulationStation.GetIiwaPosition.doc);
  station.def("SetIiwaPosition", &ManipulationStation<T>::SetIiwaPosition,
              doc.ManipulationStation.SetIiwaPosition.doc);
  station.def("GetIiwaVelocity", &ManipulationStation<T>::GetIiwaVelocity,
              doc.ManipulationStation.GetIiwaVelocity.doc);
  station.def("SetIiwaVelocity", &ManipulationStation<T>::SetIiwaVelocity,
              doc.ManipulationStation.SetIiwaVelocity.doc);
  station.def("GetWsgPosition", &ManipulationStation<T>::GetWsgPosition,
              doc.ManipulationStation.GetWsgPosition.doc);
  station.def("SetWsgPosition", &ManipulationStation<T>::SetWsgPosition,
              doc.ManipulationStation.SetWsgPosition.doc);
  station.def("GetWsgVelocity", &ManipulationStation<T>::GetWsgVelocity,
              doc.ManipulationStation.GetWsgVelocity.doc);
  station.def("SetWsgVelocity", &ManipulationStation<T>::SetWsgVelocity,
              doc.ManipulationStation.SetWsgVelocity.doc);
  station.def_static("get_camera_pose",
                     &ManipulationStation<T>::get_camera_pose,
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
