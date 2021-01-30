#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/examples/rimless_wheel/gen/rimless_wheel_continuous_state.h"
#include "drake/examples/rimless_wheel/gen/rimless_wheel_params.h"
#include "drake/examples/rimless_wheel/rimless_wheel.h"
#include "drake/examples/rimless_wheel/rimless_wheel_geometry.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(rimless_wheel, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::systems;
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::examples::rimless_wheel;
  constexpr auto& doc = pydrake_doc.drake.examples.rimless_wheel;

  m.doc() = "Bindings for the rimless_wheel example.";

  py::module::import("pydrake.systems.framework");

  // TODO(eric.cousineau): At present, we only bind doubles.
  // In the future, we will bind more scalar types, and enable scalar
  // conversion.
  using T = double;

  py::class_<RimlessWheel<T>, LeafSystem<T>>(
      m, "RimlessWheel", doc.RimlessWheel.doc)
      .def(py::init<>(), doc.RimlessWheel.ctor.doc)
      .def("get_minimal_state_output_port",
          &RimlessWheel<T>::get_minimal_state_output_port,
          py_rvp::reference_internal,
          doc.RimlessWheel.get_minimal_state_output_port.doc)
      .def("get_floating_base_state_output_port",
          &RimlessWheel<T>::get_floating_base_state_output_port,
          py_rvp::reference_internal,
          doc.RimlessWheel.get_floating_base_state_output_port.doc)
      .def_static("calc_alpha", &RimlessWheel<T>::calc_alpha, py::arg("params"),
          doc.RimlessWheel.calc_alpha.doc);

  // TODO(russt): Remove custom bindings once #8096 is resolved.
  py::class_<RimlessWheelParams<T>, BasicVector<T>>(
      m, "RimlessWheelParams", doc.RimlessWheelParams.doc)
      .def(py::init<>(), doc.RimlessWheelParams.ctor.doc)
      .def(
          "mass", &RimlessWheelParams<T>::mass, doc.RimlessWheelParams.mass.doc)
      .def("length", &RimlessWheelParams<T>::length,
          doc.RimlessWheelParams.length.doc)
      .def("gravity", &RimlessWheelParams<T>::gravity,
          doc.RimlessWheelParams.gravity.doc)
      .def("number_of_spokes", &RimlessWheelParams<T>::number_of_spokes,
          doc.RimlessWheelParams.number_of_spokes.doc)
      .def("slope", &RimlessWheelParams<T>::slope,
          doc.RimlessWheelParams.slope.doc)
      .def("set_mass", &RimlessWheelParams<T>::set_mass,
          doc.RimlessWheelParams.set_mass.doc)
      .def("set_length", &RimlessWheelParams<T>::set_length,
          doc.RimlessWheelParams.set_length.doc)
      .def("set_gravity", &RimlessWheelParams<T>::set_gravity,
          doc.RimlessWheelParams.set_gravity.doc)
      .def("set_number_of_spokes", &RimlessWheelParams<T>::set_number_of_spokes,
          doc.RimlessWheelParams.set_number_of_spokes.doc)
      .def("set_slope", &RimlessWheelParams<T>::set_slope,
          doc.RimlessWheelParams.set_slope.doc);

  py::class_<RimlessWheelContinuousState<T>, BasicVector<T>>(
      m, "RimlessWheelContinuousState", doc.RimlessWheelContinuousState.doc)
      .def(py::init<>(), doc.RimlessWheelContinuousState.ctor.doc)
      .def("theta", &RimlessWheelContinuousState<T>::theta,
          doc.RimlessWheelContinuousState.theta.doc)
      .def("thetadot", &RimlessWheelContinuousState<T>::thetadot,
          doc.RimlessWheelContinuousState.thetadot.doc)
      .def("set_theta", &RimlessWheelContinuousState<T>::set_theta,
          doc.RimlessWheelContinuousState.set_theta.doc)
      .def("set_thetadot", &RimlessWheelContinuousState<T>::set_thetadot,
          doc.RimlessWheelContinuousState.set_thetadot.doc);

  py::class_<RimlessWheelGeometry, LeafSystem<double>>(
      m, "RimlessWheelGeometry", doc.RimlessWheelGeometry.doc)
      .def_static("AddToBuilder",
          py::overload_cast<systems::DiagramBuilder<double>*,
              const systems::OutputPort<double>&,
              const RimlessWheelParams<double>&, geometry::SceneGraph<double>*>(
              &RimlessWheelGeometry::AddToBuilder),
          py::arg("builder"), py::arg("floating_base_state_port"),
          py::arg("rimless_wheel_params"), py::arg("scene_graph"),
          // Keep alive, ownership: `return` keeps `builder` alive.
          py::keep_alive<0, 1>(),
          // See #11531 for why `py_rvp::reference` is needed.
          py_rvp::reference, doc.RimlessWheelGeometry.AddToBuilder.doc_4args)
      .def_static("AddToBuilder",
          py::overload_cast<systems::DiagramBuilder<double>*,
              const systems::OutputPort<double>&,
              geometry::SceneGraph<double>*>(
              &RimlessWheelGeometry::AddToBuilder),
          py::arg("builder"), py::arg("floating_base_state_port"),
          py::arg("scene_graph"),
          // Keep alive, ownership: `return` keeps `builder` alive.
          py::keep_alive<0, 1>(),
          // See #11531 for why `py_rvp::reference` is needed.
          py_rvp::reference, doc.RimlessWheelGeometry.AddToBuilder.doc_3args);
}

}  // namespace pydrake
}  // namespace drake
