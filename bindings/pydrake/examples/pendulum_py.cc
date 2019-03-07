#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/documentation_pybind.h"
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
  constexpr auto& doc = pydrake_doc.drake.examples.pendulum;

  py::module::import("pydrake.systems.framework");

  // TODO(eric.cousineau): At present, we only bind doubles.
  // In the future, we will bind more scalar types, and enable scalar
  // conversion.
  using T = double;

  py::class_<PendulumPlant<T>, LeafSystem<T>>(
      m, "PendulumPlant", doc.PendulumPlant.doc)
      .def(py::init<>(), doc.PendulumPlant.ctor.doc);

  // TODO(russt): Remove custom bindings once #8096 is resolved.
  py::class_<PendulumInput<T>, BasicVector<T>>(
      m, "PendulumInput", doc.PendulumInput.doc)
      .def(py::init<>(), doc.PendulumInput.ctor.doc)
      .def("tau", &PendulumInput<T>::tau, doc.PendulumInput.tau.doc)
      .def(
          "set_tau", &PendulumInput<T>::set_tau, doc.PendulumInput.set_tau.doc);

  py::class_<PendulumParams<T>, BasicVector<T>>(
      m, "PendulumParams", doc.PendulumParams.doc)
      .def(py::init<>(), doc.PendulumParams.ctor.doc)
      .def("mass", &PendulumParams<T>::mass, doc.PendulumParams.mass.doc)
      .def("length", &PendulumParams<T>::length, doc.PendulumParams.length.doc)
      .def("damping", &PendulumParams<T>::damping,
          doc.PendulumParams.damping.doc)
      .def("gravity", &PendulumParams<T>::gravity,
          doc.PendulumParams.gravity.doc)
      .def("set_mass", &PendulumParams<T>::set_mass,
          doc.PendulumParams.set_mass.doc)
      .def("set_length", &PendulumParams<T>::set_length,
          doc.PendulumParams.set_length.doc)
      .def("set_damping", &PendulumParams<T>::set_damping,
          doc.PendulumParams.set_damping.doc)
      .def("set_gravity", &PendulumParams<T>::set_gravity,
          doc.PendulumParams.set_gravity.doc);

  py::class_<PendulumState<T>, BasicVector<T>>(
      m, "PendulumState", doc.PendulumState.doc)
      .def(py::init<>(), doc.PendulumState.ctor.doc)
      .def("theta", &PendulumState<T>::theta, doc.PendulumState.theta.doc)
      .def("thetadot", &PendulumState<T>::thetadot,
          doc.PendulumState.thetadot.doc)
      .def("set_theta", &PendulumState<T>::set_theta,
          doc.PendulumState.set_theta.doc)
      .def("set_thetadot", &PendulumState<T>::set_thetadot,
          doc.PendulumState.set_thetadot.doc);
}

}  // namespace pydrake
}  // namespace drake
