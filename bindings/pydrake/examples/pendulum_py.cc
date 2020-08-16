#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/examples/pendulum/gen/pendulum_input.h"
#include "drake/examples/pendulum/gen/pendulum_params.h"
#include "drake/examples/pendulum/gen/pendulum_state.h"
#include "drake/examples/pendulum/pendulum_geometry.h"
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
      .def(py::init<>(), doc.PendulumPlant.ctor.doc)
      .def("get_state_output_port", &PendulumPlant<T>::get_state_output_port,
          py_rvp::reference_internal,
          doc.PendulumPlant.get_state_output_port.doc)
      .def_static("get_state",
          py::overload_cast<const Context<T>&>(&PendulumPlant<T>::get_state),
          py::arg("context"),
          // Keey alive, ownership: `return` keeps `context` alive
          py::keep_alive<0, 1>(), doc.PendulumPlant.get_state.doc)
      .def_static("get_mutable_state",
          py::overload_cast<Context<T>*>(&PendulumPlant<T>::get_mutable_state),
          py::arg("context"),
          // Keey alive, ownership: `return` keeps `context` alive
          py::keep_alive<0, 1>(), doc.PendulumPlant.get_mutable_state.doc)
      .def("get_parameters", &PendulumPlant<T>::get_parameters,
          py_rvp::reference_internal, py::arg("context"),
          doc.PendulumPlant.get_parameters.doc)
      .def("get_mutable_parameters", &PendulumPlant<T>::get_mutable_parameters,
          py_rvp::reference_internal, py::arg("context"),
          doc.PendulumPlant.get_mutable_parameters.doc);

  // TODO(russt): Remove custom bindings once #8096 is resolved.
  py::class_<PendulumInput<T>, BasicVector<T>>(
      m, "PendulumInput", doc.PendulumInput.doc)
      .def(py::init<>(), doc.PendulumInput.ctor.doc)
      .def("tau", &PendulumInput<T>::tau, doc.PendulumInput.tau.doc)
      .def("set_tau", &PendulumInput<T>::set_tau, py::arg("tau"),
          doc.PendulumInput.set_tau.doc)
      .def("with_tau", &PendulumInput<T>::with_tau, py::arg("tau"),
          doc.PendulumInput.with_tau.doc);

  py::class_<PendulumParams<T>, BasicVector<T>>(
      m, "PendulumParams", doc.PendulumParams.doc)
      .def(py::init<>(), doc.PendulumParams.ctor.doc)
      .def("mass", &PendulumParams<T>::mass, doc.PendulumParams.mass.doc)
      .def("length", &PendulumParams<T>::length, doc.PendulumParams.length.doc)
      .def("damping", &PendulumParams<T>::damping,
          doc.PendulumParams.damping.doc)
      .def("gravity", &PendulumParams<T>::gravity,
          doc.PendulumParams.gravity.doc)
      .def("set_mass", &PendulumParams<T>::set_mass, py::arg("mass"),
          doc.PendulumParams.set_mass.doc)
      .def("set_length", &PendulumParams<T>::set_length, py::arg("length"),
          doc.PendulumParams.set_length.doc)
      .def("set_damping", &PendulumParams<T>::set_damping, py::arg("damping"),
          doc.PendulumParams.set_damping.doc)
      .def("set_gravity", &PendulumParams<T>::set_gravity, py::arg("gravity"),
          doc.PendulumParams.set_gravity.doc)
      .def("with_mass", &PendulumParams<T>::with_mass, py::arg("mass"),
          doc.PendulumParams.with_mass.doc)
      .def("with_length", &PendulumParams<T>::with_length, py::arg("length"),
          doc.PendulumParams.with_length.doc)
      .def("with_damping", &PendulumParams<T>::with_damping, py::arg("damping"),
          doc.PendulumParams.with_damping.doc)
      .def("with_gravity", &PendulumParams<T>::with_gravity, py::arg("gravity"),
          doc.PendulumParams.with_gravity.doc);

  py::class_<PendulumState<T>, BasicVector<T>>(
      m, "PendulumState", doc.PendulumState.doc)
      .def(py::init<>(), doc.PendulumState.ctor.doc)
      .def("theta", &PendulumState<T>::theta, doc.PendulumState.theta.doc)
      .def("thetadot", &PendulumState<T>::thetadot,
          doc.PendulumState.thetadot.doc)
      .def("set_theta", &PendulumState<T>::set_theta, py::arg("theta"),
          doc.PendulumState.set_theta.doc)
      .def("set_thetadot", &PendulumState<T>::set_thetadot, py::arg("thetadot"),
          doc.PendulumState.set_thetadot.doc)
      .def("with_theta", &PendulumState<T>::with_theta, py::arg("theta"),
          doc.PendulumState.with_theta.doc)
      .def("with_thetadot", &PendulumState<T>::with_thetadot,
          py::arg("thetadot"), doc.PendulumState.with_thetadot.doc);

  py::class_<PendulumGeometry, LeafSystem<double>>(
      m, "PendulumGeometry", doc.PendulumGeometry.doc)
      .def_static("AddToBuilder", &PendulumGeometry::AddToBuilder,
          py::arg("builder"), py::arg("pendulum_state_port"),
          py::arg("scene_graph"),
          // Keep alive, ownership: `return` keeps `builder` alive.
          py::keep_alive<0, 1>(),
          // See #11531 for why `py_rvp::reference` is needed.
          py_rvp::reference, doc.PendulumGeometry.AddToBuilder.doc);
}

}  // namespace pydrake
}  // namespace drake
