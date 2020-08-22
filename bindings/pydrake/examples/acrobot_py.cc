#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/examples/acrobot/acrobot_geometry.h"
#include "drake/examples/acrobot/acrobot_plant.h"
#include "drake/examples/acrobot/gen/acrobot_input.h"
#include "drake/examples/acrobot/gen/acrobot_params.h"
#include "drake/examples/acrobot/gen/acrobot_state.h"
#include "drake/examples/acrobot/gen/spong_controller_params.h"
#include "drake/examples/acrobot/spong_controller.h"

using std::make_unique;
using std::unique_ptr;
using std::vector;

namespace drake {
namespace pydrake {

PYBIND11_MODULE(acrobot, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::systems;
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::examples::acrobot;
  constexpr auto& doc = pydrake_doc.drake.examples.acrobot;

  m.doc() = "Bindings for the Acrobot example.";

  py::module::import("pydrake.systems.framework");

  // TODO(eric.cousineau): At present, we only bind doubles.
  // In the future, we will bind more scalar types, and enable scalar
  // conversion. Issue #7660.
  using T = double;

  py::class_<AcrobotPlant<T>, LeafSystem<T>>(
      m, "AcrobotPlant", doc.AcrobotPlant.doc)
      .def(py::init<>(), doc.AcrobotPlant.ctor.doc)
      .def("DynamicsBiasTerm", &AcrobotPlant<T>::DynamicsBiasTerm,
          doc.AcrobotPlant.DynamicsBiasTerm.doc)
      .def("SetMITAcrobotParameters", &AcrobotPlant<T>::SetMITAcrobotParameters,
          doc.AcrobotPlant.SetMITAcrobotParameters.doc)
      .def("MassMatrix", &AcrobotPlant<T>::MassMatrix,
          doc.AcrobotPlant.MassMatrix.doc)
      .def_static("get_state",
          py::overload_cast<const Context<T>&>(&AcrobotPlant<T>::get_state),
          py::arg("context"),
          // Keey alive, ownership: `return` keeps `context` alive
          py::keep_alive<0, 1>(), doc.AcrobotPlant.get_state.doc)
      .def_static("get_mutable_state",
          py::overload_cast<Context<T>*>(&AcrobotPlant<T>::get_mutable_state),
          py::arg("context"),
          // Keep alive, ownership: `return` keeps `context` alive
          py::keep_alive<0, 1>(), doc.AcrobotPlant.get_mutable_state.doc)
      .def("get_parameters", &AcrobotPlant<T>::get_parameters,
          py_rvp::reference_internal, py::arg("context"),
          doc.AcrobotPlant.get_parameters.doc)
      .def("get_mutable_parameters", &AcrobotPlant<T>::get_mutable_parameters,
          py_rvp::reference_internal, py::arg("context"),
          doc.AcrobotPlant.get_mutable_parameters.doc);

  py::class_<AcrobotSpongController<T>, LeafSystem<T>>(
      m, "AcrobotSpongController", doc.AcrobotSpongController.doc)
      .def(py::init<>(), doc.AcrobotSpongController.ctor.doc)
      .def("get_parameters", &AcrobotSpongController<T>::get_parameters,
          py_rvp::reference, py::arg("context"),
          // Keep alive, ownership: `return` keeps `context` alive.
          py::keep_alive<0, 2>(), doc.AcrobotSpongController.get_parameters.doc)
      .def("get_mutable_parameters",
          &AcrobotSpongController<T>::get_mutable_parameters, py_rvp::reference,
          py::arg("context"),
          // Keep alive, ownership: `return` keeps `context` alive.
          py::keep_alive<0, 2>(),
          doc.AcrobotSpongController.get_mutable_parameters.doc);

  // TODO(russt): Remove custom bindings once #8096 is resolved.
  py::class_<AcrobotInput<T>, BasicVector<T>>(
      m, "AcrobotInput", doc.AcrobotInput.doc)
      .def(py::init<>(), doc.AcrobotInput.ctor.doc)
      .def("tau", &AcrobotInput<T>::tau, doc.AcrobotInput.tau.doc)
      .def("set_tau", &AcrobotInput<T>::set_tau, doc.AcrobotInput.set_tau.doc);

  py::class_<AcrobotParams<T>, BasicVector<T>>(
      m, "AcrobotParams", doc.AcrobotParams.doc)
      .def(py::init<>(), doc.AcrobotParams.ctor.doc)
      .def("m1", &AcrobotParams<T>::m1, doc.AcrobotParams.m1.doc)
      .def("m2", &AcrobotParams<T>::m2, doc.AcrobotParams.m2.doc)
      .def("l1", &AcrobotParams<T>::l1, doc.AcrobotParams.l1.doc)
      .def("lc1", &AcrobotParams<T>::lc1, doc.AcrobotParams.lc1.doc)
      .def("lc2", &AcrobotParams<T>::lc2, doc.AcrobotParams.lc2.doc)
      .def("Ic1", &AcrobotParams<T>::Ic1, doc.AcrobotParams.Ic1.doc)
      .def("Ic2", &AcrobotParams<T>::Ic2, doc.AcrobotParams.Ic2.doc)
      .def("b1", &AcrobotParams<T>::b1, doc.AcrobotParams.b1.doc)
      .def("b2", &AcrobotParams<T>::b2, doc.AcrobotParams.b2.doc)
      .def("gravity", &AcrobotParams<T>::gravity, doc.AcrobotParams.gravity.doc)
      .def("set_m1", &AcrobotParams<T>::set_m1, doc.AcrobotParams.set_m1.doc)
      .def("set_m2", &AcrobotParams<T>::set_m2, doc.AcrobotParams.set_m2.doc)
      .def("set_l1", &AcrobotParams<T>::set_l1, doc.AcrobotParams.set_l1.doc)
      .def("set_lc1", &AcrobotParams<T>::set_lc1, doc.AcrobotParams.set_lc1.doc)
      .def("set_lc2", &AcrobotParams<T>::set_lc2, doc.AcrobotParams.set_lc2.doc)
      .def("set_Ic1", &AcrobotParams<T>::set_Ic1, doc.AcrobotParams.set_Ic1.doc)
      .def("set_Ic2", &AcrobotParams<T>::set_Ic2, doc.AcrobotParams.set_Ic2.doc)
      .def("set_b1", &AcrobotParams<T>::set_b1, doc.AcrobotParams.set_b1.doc)
      .def("set_b2", &AcrobotParams<T>::set_b2, doc.AcrobotParams.set_b2.doc)
      .def("set_gravity", &AcrobotParams<T>::set_gravity,
          doc.AcrobotParams.set_gravity.doc);

  py::class_<AcrobotState<T>, BasicVector<T>>(
      m, "AcrobotState", doc.AcrobotState.doc)
      .def(py::init<>(), doc.AcrobotState.ctor.doc)
      .def("theta1", &AcrobotState<T>::theta1, doc.AcrobotState.theta1.doc)
      .def("theta1dot", &AcrobotState<T>::theta1dot,
          doc.AcrobotState.theta1dot.doc)
      .def("theta2", &AcrobotState<T>::theta2, doc.AcrobotState.theta2.doc)
      .def("theta2dot", &AcrobotState<T>::theta2dot,
          doc.AcrobotState.theta2dot.doc)
      .def("set_theta1", &AcrobotState<T>::set_theta1,
          doc.AcrobotState.set_theta1.doc)
      .def("set_theta1dot", &AcrobotState<T>::set_theta1dot,
          doc.AcrobotState.set_theta1dot.doc)
      .def("set_theta2", &AcrobotState<T>::set_theta2,
          doc.AcrobotState.set_theta2.doc)
      .def("set_theta2dot", &AcrobotState<T>::set_theta2dot,
          doc.AcrobotState.set_theta2dot.doc);

  py::class_<SpongControllerParams<T>, BasicVector<T>>(
      m, "SpongControllerParams", doc.AcrobotParams.doc)
      .def(py::init<>(), doc.SpongControllerParams.ctor.doc)
      .def("k_e", &SpongControllerParams<T>::k_e,
          doc.SpongControllerParams.k_e.doc)
      .def("k_p", &SpongControllerParams<T>::k_p,
          doc.SpongControllerParams.k_p.doc)
      .def("k_d", &SpongControllerParams<T>::k_d,
          doc.SpongControllerParams.k_d.doc)
      .def("balancing_threshold",
          &SpongControllerParams<T>::balancing_threshold,
          doc.SpongControllerParams.balancing_threshold.doc)
      .def("set_k_e", &SpongControllerParams<T>::set_k_e,
          doc.SpongControllerParams.set_k_e.doc)
      .def("set_k_p", &SpongControllerParams<T>::set_k_p,
          doc.SpongControllerParams.set_k_p.doc)
      .def("set_k_d", &SpongControllerParams<T>::set_k_d,
          doc.SpongControllerParams.set_k_d.doc)
      .def("set_balancing_threshold",
          &SpongControllerParams<T>::set_balancing_threshold,
          doc.SpongControllerParams.set_balancing_threshold.doc);

  py::class_<AcrobotGeometry, LeafSystem<double>>(
      m, "AcrobotGeometry", doc.AcrobotGeometry.doc)
      .def_static("AddToBuilder",
          py::overload_cast<systems::DiagramBuilder<double>*,
              const systems::OutputPort<double>&, const AcrobotParams<double>&,
              geometry::SceneGraph<double>*>(&AcrobotGeometry::AddToBuilder),
          py::arg("builder"), py::arg("acrobot_state_port"),
          py::arg("acrobot_params"), py::arg("scene_graph"),
          // Keep alive, ownership: `return` keeps `builder` alive.
          py::keep_alive<0, 1>(),
          // See #11531 for why `py_rvp::reference` is needed.
          py_rvp::reference, doc.AcrobotGeometry.AddToBuilder.doc_4args)
      .def_static("AddToBuilder",
          py::overload_cast<systems::DiagramBuilder<double>*,
              const systems::OutputPort<double>&,
              geometry::SceneGraph<double>*>(&AcrobotGeometry::AddToBuilder),
          py::arg("builder"), py::arg("acrobot_state_port"),
          py::arg("scene_graph"),
          // Keep alive, ownership: `return` keeps `builder` alive.
          py::keep_alive<0, 1>(),
          // See #11531 for why `py_rvp::reference` is needed.
          py_rvp::reference, doc.AcrobotGeometry.AddToBuilder.doc_3args);
}

}  // namespace pydrake
}  // namespace drake
