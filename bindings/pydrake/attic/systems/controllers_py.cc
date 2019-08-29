#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/controllers/rbt_inverse_dynamics.h"
#include "drake/systems/controllers/rbt_inverse_dynamics_controller.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(controllers, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::systems::controllers;
  constexpr auto& doc = pydrake_doc.drake.systems.controllers;
  using drake::systems::Diagram;
  using drake::systems::LeafSystem;

  py::module::import("pydrake.attic.multibody.rigid_body_tree");
  py::module::import("pydrake.systems.framework");

  py::class_<rbt::InverseDynamics<double>, LeafSystem<double>> rbt_idyn(
      m, "RbtInverseDynamics", doc.rbt.InverseDynamics.doc);
  rbt_idyn  // BR
      .def(py::init<const RigidBodyTree<double>*,
               rbt::InverseDynamics<double>::InverseDynamicsMode>(),
          py::arg("tree"), py::arg("mode"), doc.rbt.InverseDynamics.ctor.doc)
      .def("is_pure_gravity_compensation",
          &rbt::InverseDynamics<double>::is_pure_gravity_compensation,
          doc.rbt.InverseDynamics.is_pure_gravity_compensation.doc);

  py::enum_<rbt::InverseDynamics<double>::InverseDynamicsMode>(  // BR
      rbt_idyn, "InverseDynamicsMode")
      .value("kInverseDynamics", rbt::InverseDynamics<double>::kInverseDynamics,
          doc.rbt.InverseDynamics.InverseDynamicsMode.doc)
      .value("kGravityCompensation",
          rbt::InverseDynamics<double>::kGravityCompensation,
          doc.rbt.InverseDynamics.InverseDynamicsMode.kGravityCompensation.doc)
      .export_values();

  py::class_<rbt::InverseDynamicsController<double>, Diagram<double>>(
      m, "RbtInverseDynamicsController", doc.rbt.InverseDynamicsController.doc)
      .def(py::init<std::unique_ptr<RigidBodyTree<double>>,
               const VectorX<double>&, const VectorX<double>&,
               const VectorX<double>&, bool>(),
          py::arg("robot"), py::arg("kp"), py::arg("ki"), py::arg("kd"),
          py::arg("has_reference_acceleration"),
          // Keep alive, ownership: `robot` keeps `self` alive.
          py::keep_alive<2, 1>(), doc.rbt.InverseDynamicsController.ctor.doc)
      .def("set_integral_value",
          &rbt::InverseDynamicsController<double>::set_integral_value,
          doc.rbt.InverseDynamicsController.set_integral_value.doc);
}

}  // namespace pydrake
}  // namespace drake
