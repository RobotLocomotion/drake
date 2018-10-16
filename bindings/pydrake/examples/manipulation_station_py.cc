#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/examples/manipulation_station/station_simulation.h"

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

  // StationSimulation currently only supports double.
  using T = double;

  py::class_<StationSimulation<T>, Diagram<T>>(m, "StationSimulation")
      .def(py::init<double>(), py::arg("time_step") = 0.002,
           doc.StationSimulation.ctor.doc_3)
      .def("Finalize", &StationSimulation<T>::Finalize,
           doc.StationSimulation.Finalize.doc)
      .def("get_mutable_multibody_plant",
           &StationSimulation<T>::get_mutable_multibody_plant,
           py_reference_internal,
           doc.StationSimulation.get_mutable_multibody_plant.doc)
      .def("get_mutable_scene_graph",
           &StationSimulation<T>::get_mutable_scene_graph,
           py_reference_internal,
           doc.StationSimulation.get_mutable_scene_graph.doc)
      .def("get_controller_plant",
           &StationSimulation<T>::get_controller_plant,
           py_reference_internal,
           doc.StationSimulation.get_controller_plant.doc)
      .def("GetIiwaPosition", &StationSimulation<T>::GetIiwaPosition,
           doc.StationSimulation.GetIiwaPosition.doc)
      .def("SetIiwaPosition", &StationSimulation<T>::SetIiwaPosition,
           doc.StationSimulation.SetIiwaPosition.doc)
      .def("GetIiwaVelocity", &StationSimulation<T>::GetIiwaVelocity,
           doc.StationSimulation.GetIiwaVelocity.doc)
      .def("SetIiwaVelocity", &StationSimulation<T>::SetIiwaVelocity,
           doc.StationSimulation.SetIiwaVelocity.doc);
}

}  // namespace pydrake
}  // namespace drake
