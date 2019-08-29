#include "pybind11/eigen.h"
#include "pybind11/functional.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "drake/bindings/pydrake/common/wrap_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/symbolic_types_pybind.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/controllers/dynamic_programming.h"
#include "drake/systems/controllers/inverse_dynamics.h"
#include "drake/systems/controllers/inverse_dynamics_controller.h"
#include "drake/systems/controllers/linear_quadratic_regulator.h"

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
  py::module::import("pydrake.multibody.plant");
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
          // Keep alive, reference: `self` keeps `robot` alive.
          py::keep_alive<1, 2>(), doc.InverseDynamicsController.ctor.doc)
      .def("set_integral_value",
          &InverseDynamicsController<double>::set_integral_value,
          doc.InverseDynamicsController.set_integral_value.doc)
      .def("get_input_port_desired_acceleration",
          &InverseDynamicsController<
              double>::get_input_port_desired_acceleration,
          py_reference_internal,
          doc.InverseDynamicsController.get_input_port_desired_acceleration.doc)
      .def("get_input_port_estimated_state",
          &InverseDynamicsController<double>::get_input_port_estimated_state,
          py_reference_internal,
          doc.InverseDynamicsController.get_input_port_estimated_state.doc)
      .def("get_input_port_desired_state",
          &InverseDynamicsController<double>::get_input_port_desired_state,
          py_reference_internal,
          doc.InverseDynamicsController.get_input_port_desired_state.doc)
      .def("get_output_port_control",
          &InverseDynamicsController<double>::get_output_port_control,
          py_reference_internal,
          doc.InverseDynamicsController.get_output_port_control.doc);

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
      doc.LinearQuadraticRegulator.doc_5args);

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
      doc.LinearQuadraticRegulator.doc_4args);

  m.def("LinearQuadraticRegulator",
      py::overload_cast<const systems::System<double>&,
          const systems::Context<double>&,
          const Eigen::Ref<const Eigen::MatrixXd>&,
          const Eigen::Ref<const Eigen::MatrixXd>&,
          const Eigen::Ref<const Eigen::MatrixXd>&, int>(
          &LinearQuadraticRegulator),
      py::arg("system"), py::arg("context"), py::arg("Q"), py::arg("R"),
      py::arg("N") = Eigen::Matrix<double, 0, 0>::Zero(),
      py::arg("input_port_index") = 0, doc.LinearQuadraticRegulator.doc_6args);
}

}  // namespace pydrake
}  // namespace drake
