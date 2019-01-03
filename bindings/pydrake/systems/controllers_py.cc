#include "pybind11/eigen.h"
#include "pybind11/functional.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/symbolic_types_pybind.h"
#include "drake/bindings/pydrake/util/wrap_pybind.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/controllers/dynamic_programming.h"
#include "drake/systems/controllers/inverse_dynamics.h"
#include "drake/systems/controllers/inverse_dynamics_controller.h"
#include "drake/systems/controllers/linear_quadratic_regulator.h"
#include "drake/systems/controllers/rbt_inverse_dynamics.h"
#include "drake/systems/controllers/rbt_inverse_dynamics_controller.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(controllers, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::systems::controllers;
  using drake::multibody::MultibodyPlant;
  using drake::systems::Diagram;
  using drake::systems::LeafSystem;
  constexpr auto& doc = pydrake_doc.drake.systems.controllers;

  py::module::import("pydrake.math");
  py::module::import("pydrake.symbolic");
  py::module::import("pydrake.systems.framework");
  py::module::import("pydrake.systems.primitives");

  py::class_<DynamicProgrammingOptions::PeriodicBoundaryCondition>(m,
      "PeriodicBoundaryCondition",
      doc.DynamicProgrammingOptions.PeriodicBoundaryCondition.doc)
      .def(py::init<int, double, double>(),
          doc.DynamicProgrammingOptions.PeriodicBoundaryCondition.ctor.doc);

  py::class_<DynamicProgrammingOptions>(
      m, "DynamicProgrammingOptions", doc.DynamicProgrammingOptions.doc)
      .def(py::init<>())
      .def_readwrite("discount_factor",
          &DynamicProgrammingOptions::discount_factor,
          doc.DynamicProgrammingOptions.discount_factor.doc)
      .def_readwrite("periodic_boundary_conditions",
          &DynamicProgrammingOptions::periodic_boundary_conditions,
          doc.DynamicProgrammingOptions.periodic_boundary_conditions.doc)
      .def_readwrite("convergence_tol",
          &DynamicProgrammingOptions::convergence_tol,
          doc.DynamicProgrammingOptions.convergence_tol.doc)
      .def_readwrite("visualization_callback",
          &DynamicProgrammingOptions::visualization_callback,
          doc.DynamicProgrammingOptions.visualization_callback.doc);

  // The RBT flavor of inverse dynamics.

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
          // Keep alive, ownership: RigidBodyTree keeps this alive.
          // See "Keep Alive Behavior" in pydrake_pybind.h for details.
          py::keep_alive<2 /* Nurse */, 1 /* Patient */>(),
          doc.rbt.InverseDynamicsController.ctor.doc)
      .def("set_integral_value",
          &rbt::InverseDynamicsController<double>::set_integral_value,
          doc.rbt.InverseDynamicsController.set_integral_value.doc);

  // The MBP flavor of inverse dynamics.

  py::class_<InverseDynamics<double>, LeafSystem<double>> idyn(
      m, "InverseDynamics", doc.InverseDynamics.doc);
  idyn  // BR
      .def(py::init<const multibody::MultibodyPlant<double>*,
               InverseDynamics<double>::InverseDynamicsMode>(),
          py::arg("plant"), py::arg("mode"), doc.InverseDynamics.ctor.doc)
      .def("is_pure_gravity_compensation",
          &InverseDynamics<double>::is_pure_gravity_compensation,
          doc.InverseDynamics.is_pure_gravity_compensation.doc);

  py::enum_<InverseDynamics<double>::InverseDynamicsMode>(  // BR
      idyn, "InverseDynamicsMode")
      .value("kInverseDynamics", InverseDynamics<double>::kInverseDynamics,
          doc.InverseDynamics.InverseDynamicsMode.doc)
      .value("kGravityCompensation",
          InverseDynamics<double>::kGravityCompensation,
          doc.InverseDynamics.InverseDynamicsMode.kGravityCompensation.doc)
      .export_values();

  py::class_<InverseDynamicsController<double>, Diagram<double>>(
      m, "InverseDynamicsController", doc.InverseDynamicsController.doc)
      .def(py::init<const MultibodyPlant<double>&, const VectorX<double>&,
               const VectorX<double>&, const VectorX<double>&, bool>(),
          py::arg("robot"), py::arg("kp"), py::arg("ki"), py::arg("kd"),
          py::arg("has_reference_acceleration"),
          // Keep alive, reference: `self` keeps `MultibodyPlant` alive.
          py::keep_alive<1, 2>(), doc.InverseDynamicsController.ctor.doc)
      .def("set_integral_value",
          &InverseDynamicsController<double>::set_integral_value,
          doc.InverseDynamicsController.set_integral_value.doc);

  m.def("FittedValueIteration", WrapCallbacks(&FittedValueIteration),
      doc.FittedValueIteration.doc);

  m.def("LinearProgrammingApproximateDynamicProgramming",
      WrapCallbacks(&LinearProgrammingApproximateDynamicProgramming),
      doc.LinearProgrammingApproximateDynamicProgramming.doc);

  m.def("LinearQuadraticRegulator",
      [](const Eigen::Ref<const Eigen::MatrixXd>& A,
          const Eigen::Ref<const Eigen::MatrixXd>& B,
          const Eigen::Ref<const Eigen::MatrixXd>& Q,
          const Eigen::Ref<const Eigen::MatrixXd>& R,
          const Eigen::Ref<const Eigen::MatrixXd>& N) {
        auto result = LinearQuadraticRegulator(A, B, Q, R, N);
        return std::make_pair(result.K, result.S);
      },
      py::arg("A"), py::arg("B"), py::arg("Q"), py::arg("R"),
      py::arg("N") = Eigen::Matrix<double, 0, 0>::Zero(),
      doc.LinearQuadraticRegulator.doc_5args_A_B_Q_R_N);

  m.def("DiscreteTimeLinearQuadraticRegulator",
      [](const Eigen::Ref<const Eigen::MatrixXd>& A,
          const Eigen::Ref<const Eigen::MatrixXd>& B,
          const Eigen::Ref<const Eigen::MatrixXd>& Q,
          const Eigen::Ref<const Eigen::MatrixXd>& R) {
        auto result = DiscreteTimeLinearQuadraticRegulator(A, B, Q, R);
        return std::make_pair(result.K, result.S);
      },
      py::arg("A"), py::arg("B"), py::arg("Q"), py::arg("R"),
      doc.DiscreteTimeLinearQuadraticRegulator.doc);

  m.def("LinearQuadraticRegulator",
      py::overload_cast<const systems::LinearSystem<double>&,
          const Eigen::Ref<const Eigen::MatrixXd>&,
          const Eigen::Ref<const Eigen::MatrixXd>&,
          const Eigen::Ref<const Eigen::MatrixXd>&>(&LinearQuadraticRegulator),
      py::arg("system"), py::arg("Q"), py::arg("R"),
      py::arg("N") = Eigen::Matrix<double, 0, 0>::Zero(),
      doc.LinearQuadraticRegulator.doc_4args_system_Q_R_N);

  m.def("LinearQuadraticRegulator",
      py::overload_cast<const systems::System<double>&,
          const systems::Context<double>&,
          const Eigen::Ref<const Eigen::MatrixXd>&,
          const Eigen::Ref<const Eigen::MatrixXd>&,
          const Eigen::Ref<const Eigen::MatrixXd>&>(&LinearQuadraticRegulator),
      py::arg("system"), py::arg("context"), py::arg("Q"), py::arg("R"),
      py::arg("N") = Eigen::Matrix<double, 0, 0>::Zero(),
      doc.LinearQuadraticRegulator.doc_5args_system_context_Q_R_N);
}

}  // namespace pydrake
}  // namespace drake
