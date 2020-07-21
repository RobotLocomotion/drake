#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/examples/compass_gait/compass_gait.h"
#include "drake/examples/compass_gait/compass_gait_geometry.h"
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
  constexpr auto& doc = pydrake_doc.drake.examples.compass_gait;

  py::module::import("pydrake.systems.framework");

  // TODO(eric.cousineau): At present, we only bind doubles.
  // In the future, we will bind more scalar types, and enable scalar
  // conversion.
  using T = double;

  py::class_<CompassGait<T>, LeafSystem<T>>(
      m, "CompassGait", doc.CompassGait.doc)
      .def(py::init<>(), doc.CompassGait.ctor.doc)
      .def("get_minimal_state_output_port",
          &CompassGait<T>::get_minimal_state_output_port,
          py_rvp::reference_internal,
          doc.CompassGait.get_minimal_state_output_port.doc)
      .def("get_floating_base_state_output_port",
          &CompassGait<T>::get_floating_base_state_output_port,
          py_rvp::reference_internal,
          doc.CompassGait.get_floating_base_state_output_port.doc);

  // TODO(russt): Remove custom bindings once #8096 is resolved.
  py::class_<CompassGaitParams<T>, BasicVector<T>>(
      m, "CompassGaitParams", doc.CompassGaitParams.doc)
      .def(py::init<>(), doc.CompassGaitParams.ctor.doc)
      .def("mass_hip", &CompassGaitParams<T>::mass_hip,
          doc.CompassGaitParams.mass_hip.doc)
      .def("mass_leg", &CompassGaitParams<T>::mass_leg,
          doc.CompassGaitParams.mass_leg.doc)
      .def("length_leg", &CompassGaitParams<T>::length_leg,
          doc.CompassGaitParams.length_leg.doc)
      .def("center_of_mass_leg", &CompassGaitParams<T>::center_of_mass_leg,
          doc.CompassGaitParams.center_of_mass_leg.doc)
      .def("gravity", &CompassGaitParams<T>::gravity,
          doc.CompassGaitParams.gravity.doc)
      .def("slope", &CompassGaitParams<T>::slope,
          doc.CompassGaitParams.slope.doc)
      .def("set_mass_hip", &CompassGaitParams<T>::set_mass_hip,
          doc.CompassGaitParams.set_mass_hip.doc)
      .def("set_mass_leg", &CompassGaitParams<T>::set_mass_leg,
          doc.CompassGaitParams.set_mass_leg.doc)
      .def("set_length_leg", &CompassGaitParams<T>::set_length_leg,
          doc.CompassGaitParams.set_length_leg.doc)
      .def("set_center_of_mass_leg",
          &CompassGaitParams<T>::set_center_of_mass_leg,
          doc.CompassGaitParams.set_center_of_mass_leg.doc)
      .def("set_gravity", &CompassGaitParams<T>::set_gravity,
          doc.CompassGaitParams.set_gravity.doc)
      .def("set_slope", &CompassGaitParams<T>::set_slope,
          doc.CompassGaitParams.set_slope.doc);

  py::class_<CompassGaitContinuousState<T>, BasicVector<T>>(
      m, "CompassGaitContinuousState", doc.CompassGaitContinuousState.doc)
      .def(py::init<>(), doc.CompassGaitContinuousState.ctor.doc)
      .def("stance", &CompassGaitContinuousState<T>::stance,
          doc.CompassGaitContinuousState.stance.doc)
      .def("swing", &CompassGaitContinuousState<T>::swing,
          doc.CompassGaitContinuousState.swing.doc)
      .def("stancedot", &CompassGaitContinuousState<T>::stancedot,
          doc.CompassGaitContinuousState.stancedot.doc)
      .def("swingdot", &CompassGaitContinuousState<T>::swingdot,
          doc.CompassGaitContinuousState.swingdot.doc)
      .def("set_stance", &CompassGaitContinuousState<T>::set_stance,
          doc.CompassGaitContinuousState.set_stance.doc)
      .def("set_swing", &CompassGaitContinuousState<T>::set_swing,
          doc.CompassGaitContinuousState.set_swing.doc)
      .def("set_stancedot", &CompassGaitContinuousState<T>::set_stancedot,
          doc.CompassGaitContinuousState.set_stancedot.doc)
      .def("set_swingdot", &CompassGaitContinuousState<T>::set_swingdot,
          doc.CompassGaitContinuousState.set_swingdot.doc);

  py::class_<CompassGaitGeometry, LeafSystem<double>>(
      m, "CompassGaitGeometry", doc.CompassGaitGeometry.doc)
      .def_static("AddToBuilder",
          py::overload_cast<systems::DiagramBuilder<double>*,
              const systems::OutputPort<double>&,
              const CompassGaitParams<double>&, geometry::SceneGraph<double>*>(
              &CompassGaitGeometry::AddToBuilder),
          py::arg("builder"), py::arg("floating_base_state_port"),
          py::arg("compass_gait_params"), py::arg("scene_graph"),
          // Keep alive, ownership: `return` keeps `builder` alive.
          py::keep_alive<0, 1>(),
          // See #11531 for why `py_rvp::reference` is needed.
          py_rvp::reference, doc.CompassGaitGeometry.AddToBuilder.doc_4args)
      .def_static("AddToBuilder",
          py::overload_cast<systems::DiagramBuilder<double>*,
              const systems::OutputPort<double>&,
              geometry::SceneGraph<double>*>(
              &CompassGaitGeometry::AddToBuilder),
          py::arg("builder"), py::arg("floating_base_state_port"),
          py::arg("scene_graph"),
          // Keep alive, ownership: `return` keeps `builder` alive.
          py::keep_alive<0, 1>(),
          // See #11531 for why `py_rvp::reference` is needed.
          py_rvp::reference, doc.CompassGaitGeometry.AddToBuilder.doc_3args);
}

}  // namespace pydrake
}  // namespace drake
