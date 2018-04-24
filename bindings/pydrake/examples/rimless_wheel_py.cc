#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/examples/rimless_wheel/gen/rimless_wheel_continuous_state.h"
#include "drake/examples/rimless_wheel/gen/rimless_wheel_params.h"
#include "drake/examples/rimless_wheel/rimless_wheel.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(rimless_wheel, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::systems;
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::examples::rimless_wheel;

  m.doc() = "Bindings for the rimless_wheel example.";

  py::module::import("pydrake.systems.framework");

  // TODO(eric.cousineau): At present, we only bind doubles.
  // In the future, we will bind more scalar types, and enable scalar
  // conversion.
  using T = double;

  py::class_<RimlessWheel<T>, LeafSystem<T>>(m, "RimlessWheel")
      .def(py::init<>());

  // TODO(russt): Remove custom bindings once #8096 is resolved.
  py::class_<RimlessWheelParams<T>, BasicVector<T>>(m, "RimlessWheelParams")
      .def(py::init<>())
      .def("mass", &RimlessWheelParams<T>::mass)
      .def("length", &RimlessWheelParams<T>::length)
      .def("gravity", &RimlessWheelParams<T>::gravity)
      .def("number_of_spokes", &RimlessWheelParams<T>::number_of_spokes)
      .def("slope", &RimlessWheelParams<T>::slope)
      .def("set_mass", &RimlessWheelParams<T>::set_mass)
      .def("set_length", &RimlessWheelParams<T>::set_length)
      .def("set_gravity", &RimlessWheelParams<T>::set_gravity)
      .def("set_number_of_spokes", &RimlessWheelParams<T>::set_number_of_spokes)
      .def("set_slope", &RimlessWheelParams<T>::set_slope);

  py::class_<RimlessWheelContinuousState<T>, BasicVector<T>>(
      m, "RimlessWheelContinuousState")
      .def(py::init<>())
      .def("theta", &RimlessWheelContinuousState<T>::theta)
      .def("thetadot", &RimlessWheelContinuousState<T>::thetadot)
      .def("set_theta", &RimlessWheelContinuousState<T>::set_theta)
      .def("set_thetadot", &RimlessWheelContinuousState<T>::set_thetadot);
}

}  // namespace pydrake
}  // namespace drake
