#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/examples/compass_gait/compass_gait.h"
#include "drake/examples/compass_gait/gen/compass_gait_continuous_state.h"
#include "drake/examples/compass_gait/gen/compass_gait_params.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(compass_gait, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::systems;
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::examples::compass_gait;

  m.doc() = "Bindings for the compass gait example.";

  py::module::import("pydrake.systems.framework");

  // TODO(eric.cousineau): At present, we only bind doubles.
  // In the future, we will bind more scalar types, and enable scalar
  // conversion.
  using T = double;

  py::class_<CompassGait<T>, LeafSystem<T>>(m, "CompassGait").def(py::init<>());

  // TODO(russt): Remove custom bindings once #8096 is resolved.
  py::class_<CompassGaitParams<T>, BasicVector<T>>(m, "CompassGaitParams")
      .def(py::init<>())
      .def("mass_hip", &CompassGaitParams<T>::mass_hip)
      .def("mass_leg", &CompassGaitParams<T>::mass_leg)
      .def("length_leg", &CompassGaitParams<T>::length_leg)
      .def("center_of_mass_leg", &CompassGaitParams<T>::center_of_mass_leg)
      .def("gravity", &CompassGaitParams<T>::gravity)
      .def("slope", &CompassGaitParams<T>::slope)
      .def("set_mass_hip", &CompassGaitParams<T>::set_mass_hip)
      .def("set_mass_leg", &CompassGaitParams<T>::set_mass_leg)
      .def("set_length_leg", &CompassGaitParams<T>::set_length_leg)
      .def("set_center_of_mass_leg",
           &CompassGaitParams<T>::set_center_of_mass_leg)
      .def("set_gravity", &CompassGaitParams<T>::set_gravity)
      .def("set_slope", &CompassGaitParams<T>::set_slope);

  py::class_<CompassGaitContinuousState<T>, BasicVector<T>>(
      m, "CompassGaitContinuousState")
      .def(py::init<>())
      .def("stance", &CompassGaitContinuousState<T>::stance)
      .def("swing", &CompassGaitContinuousState<T>::swing)
      .def("stancedot", &CompassGaitContinuousState<T>::stancedot)
      .def("swingdot", &CompassGaitContinuousState<T>::swingdot)
      .def("set_stance", &CompassGaitContinuousState<T>::set_stance)
      .def("set_swing", &CompassGaitContinuousState<T>::set_swing)
      .def("set_stancedot", &CompassGaitContinuousState<T>::set_stancedot)
      .def("set_swingdot", &CompassGaitContinuousState<T>::set_swingdot);
}

}  // namespace pydrake
}  // namespace drake
