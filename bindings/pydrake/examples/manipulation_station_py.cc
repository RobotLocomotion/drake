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
//  constexpr auto& doc = pydrake_doc.drake.examples.manipulation_station;

  py::module::import("pydrake.systems.framework");

  // TODO(eric.cousineau): At present, we only bind doubles.
  // In the future, we will bind more scalar types, and enable scalar
  // conversion.
  using T = double;

  py::class_<StationSimulation<T>, Diagram<T>>(m, "StationSimulation")
      .def(py::init<double>(), py::arg("time_step") = 0.002)
      .def("Finalize", &StationSimulation<T>::Finalize)
      .def("get_mutable_multibody_plant",
           &StationSimulation<T>::get_mutable_multibody_plant,
           py_reference_internal)
      .def("get_mutable_scene_graph",
           &StationSimulation<T>::get_mutable_scene_graph,
           py_reference_internal)
      .def("get_controller_plant",
           &StationSimulation<T>::get_controller_plant,
           py_reference_internal)
      .def("GetIiwaPosition", &StationSimulation<T>::GetIiwaPosition)
      .def("SetIiwaPosition", &StationSimulation<T>::SetIiwaPosition)
      .def("GetIiwaVelocity", &StationSimulation<T>::GetIiwaVelocity)
      .def("SetIiwaVelocity", &StationSimulation<T>::SetIiwaVelocity);
}

}  // namespace pydrake
}  // namespace drake
