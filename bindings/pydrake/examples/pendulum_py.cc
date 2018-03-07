#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/examples/pendulum/gen/pendulum_input.h"
#include "drake/examples/pendulum/gen/pendulum_params.h"
#include "drake/examples/pendulum/gen/pendulum_state.h"
#include "drake/examples/pendulum/pendulum_plant.h"

using std::make_unique;
using std::unique_ptr;
using std::vector;

namespace drake {
namespace pydrake {

PYBIND11_MODULE(pendulum, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::systems;
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::examples::pendulum;

  m.doc() = "Bindings for the Pendulum example.";

  py::module::import("pydrake.systems.framework");

  // TODO(eric.cousineau): At present, we only bind doubles.
  // In the future, we will bind more scalar types, and enable scalar
  // conversion.
  using T = double;

  py::class_<PendulumPlant<T>, LeafSystem<T>>(m, "PendulumPlant")
    .def(py::init<>());

  // TODO(russt): Remove custom bindings once #8096 is resolved.
  py::class_<PendulumInput<T>, BasicVector<T>>(m, "PendulumInput")
      .def(py::init<>())
      .def("tau", &PendulumInput<T>::tau)
      .def("set_tau", &PendulumInput<T>::set_tau);

  py::class_<PendulumParams<T>, BasicVector<T>>(m, "PendulumParams")
    .def(py::init<>())
    .def("mass", &PendulumParams<T>::mass)
    .def("length", &PendulumParams<T>::length)
    .def("damping", &PendulumParams<T>::damping)
    .def("gravity", &PendulumParams<T>::gravity)
    .def("set_mass", &PendulumParams<T>::set_mass)
    .def("set_length", &PendulumParams<T>::set_length)
    .def("set_damping", &PendulumParams<T>::set_damping)
    .def("set_gravity", &PendulumParams<T>::set_gravity);

  py::class_<PendulumState<T>, BasicVector<T>>(m, "PendulumState")
      .def(py::init<>())
      .def("theta", &PendulumState<T>::theta)
      .def("thetadot", &PendulumState<T>::thetadot)
      .def("set_theta", &PendulumState<T>::set_theta)
      .def("set_thetadot", &PendulumState<T>::set_thetadot);
}

}  // namespace pydrake
}  // namespace drake
