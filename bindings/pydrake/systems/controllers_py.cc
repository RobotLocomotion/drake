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
#include "drake/systems/controllers/finite_horizon_linear_quadratic_regulator.h"
#include "drake/systems/controllers/inverse_dynamics.h"
#include "drake/systems/controllers/inverse_dynamics_controller.h"
#include "drake/systems/controllers/linear_quadratic_regulator.h"
#include "drake/systems/controllers/pid_controlled_system.h"
#include "drake/systems/controllers/pid_controller.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(controllers, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::systems::controllers;
  using drake::multibody::MultibodyPlant;
  using drake::systems::Diagram;
  using drake::systems::LeafSystem;
  using drake::systems::System;
  constexpr auto& doc = pydrake_doc.drake.systems.controllers;

  py::module::import("pydrake.math");
  py::module::import("pydrake.multibody.plant");
  py::module::import("pydrake.symbolic");
  py::module::import("pydrake.systems.framework");
  py::module::import("pydrake.systems.primitives");
  py::module::import("pydrake.trajectories");

  py::class_<DynamicProgrammingOptions::PeriodicBoundaryCondition>(m,
      "PeriodicBoundaryCondition",
      doc.DynamicProgrammingOptions.PeriodicBoundaryCondition.doc)
      .def(py::init<int, double, double>(),
          doc.DynamicProgrammingOptions.PeriodicBoundaryCondition.ctor.doc);

  py::class_<DynamicProgrammingOptions>(
      m, "DynamicProgrammingOptions", doc.DynamicProgrammingOptions.doc)
      .def(py::init<>(), doc.DynamicProgrammingOptions.ctor.doc)
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
          doc.DynamicProgrammingOptions.visualization_callback.doc)
      .def_readwrite("input_port_index",
          &DynamicProgrammingOptions::input_port_index,
          doc.DynamicProgrammingOptions.input_port_index.doc)
      .def_readwrite("assume_non_continuous_states_are_fixed",
          &DynamicProgrammingOptions::assume_non_continuous_states_are_fixed,
          doc.DynamicProgrammingOptions.assume_non_continuous_states_are_fixed
              .doc);

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

  // TODO(eric.cousineau): Expose multiple inheritance from
  // StateFeedbackControllerInterface once #9243 is resolved.
  py::class_<InverseDynamicsController<double>, Diagram<double>>(
      m, "InverseDynamicsController", doc.InverseDynamicsController.doc)
      .def(py::init<const MultibodyPlant<double>&, const VectorX<double>&,
               const VectorX<double>&, const VectorX<double>&, bool>(),
          py::arg("robot"), py::arg("kp"), py::arg("ki"), py::arg("kd"),
          py::arg("has_reference_acceleration"),
          // Keep alive, reference: `self` keeps `robot` alive.
          py::keep_alive<1, 2>(),
          doc.InverseDynamicsController.ctor.doc_5args_referenced_plant)
      .def("set_integral_value",
          &InverseDynamicsController<double>::set_integral_value,
          doc.InverseDynamicsController.set_integral_value.doc)
      .def("get_input_port_desired_acceleration",
          &InverseDynamicsController<
              double>::get_input_port_desired_acceleration,
          py_rvp::reference_internal,
          doc.InverseDynamicsController.get_input_port_desired_acceleration.doc)
      .def("get_input_port_estimated_state",
          &InverseDynamicsController<double>::get_input_port_estimated_state,
          py_rvp::reference_internal,
          doc.InverseDynamicsController.get_input_port_estimated_state.doc)
      .def("get_input_port_desired_state",
          &InverseDynamicsController<double>::get_input_port_desired_state,
          py_rvp::reference_internal,
          doc.InverseDynamicsController.get_input_port_desired_state.doc)
      .def("get_output_port_control",
          &InverseDynamicsController<double>::get_output_port_control,
          py_rvp::reference_internal,
          doc.InverseDynamicsController.get_output_port_control.doc);

  py::class_<PidControlledSystem<double>, Diagram<double>>(
      m, "PidControlledSystem", doc.PidControlledSystem.doc)
      .def(py::init<std::unique_ptr<System<double>>, double, double, double,
               int, int>(),
          py::arg("plant"), py::arg("kp"), py::arg("ki"), py::arg("kd"),
          py::arg("state_output_port_index") = 0,
          py::arg("plant_input_port_index") = 0,
          // Keep alive, ownership: `plant` keeps `self` alive.
          py::keep_alive<2, 1>(),
          doc.PidControlledSystem.ctor.doc_6args_double_gains)
      .def(py::init<std::unique_ptr<System<double>>, const VectorX<double>&,
               const VectorX<double>&, const VectorX<double>&, int, int>(),
          py::arg("plant"), py::arg("kp"), py::arg("ki"), py::arg("kd"),
          py::arg("state_output_port_index") = 0,
          py::arg("plant_input_port_index") = 0,
          // Keep alive, ownership: `plant` keeps `self` alive.
          py::keep_alive<2, 1>(),
          doc.PidControlledSystem.ctor.doc_6args_vector_gains)
      .def(py::init<std::unique_ptr<System<double>>, const MatrixX<double>&,
               double, double, double, int, int>(),
          py::arg("plant"), py::arg("feedback_selector"), py::arg("kp"),
          py::arg("ki"), py::arg("kd"), py::arg("state_output_port_index") = 0,
          py::arg("plant_input_port_index") = 0,
          // Keep alive, ownership: `plant` keeps `self` alive.
          py::keep_alive<2, 1>(),
          doc.PidControlledSystem.ctor.doc_7args_double_gains)
      .def(py::init<std::unique_ptr<System<double>>, const MatrixX<double>&,
               const VectorX<double>&, const VectorX<double>&,
               const VectorX<double>&, int, int>(),
          py::arg("plant"), py::arg("feedback_selector"), py::arg("kp"),
          py::arg("ki"), py::arg("kd"), py::arg("state_output_port_index") = 0,
          py::arg("plant_input_port_index") = 0,
          // Keep alive, ownership: `plant` keeps `self` alive.
          py::keep_alive<2, 1>(),
          doc.PidControlledSystem.ctor.doc_7args_vector_gains)
      .def("get_control_input_port",
          &PidControlledSystem<double>::get_control_input_port,
          py_rvp::reference_internal,
          doc.PidControlledSystem.get_control_input_port.doc)
      .def("get_state_input_port",
          &PidControlledSystem<double>::get_state_input_port,
          py_rvp::reference_internal,
          doc.PidControlledSystem.get_state_input_port.doc)
      .def("get_state_output_port",
          &PidControlledSystem<double>::get_state_output_port,
          py_rvp::reference_internal,
          doc.PidControlledSystem.get_state_output_port.doc);

  // TODO(eric.cousineau): Expose multiple inheritance from
  // StateFeedbackControllerInterface once #9243 is resolved.
  py::class_<PidController<double>, LeafSystem<double>>(
      m, "PidController", doc.PidController.doc)
      .def(py::init<const VectorX<double>&, const VectorX<double>&,
               const VectorX<double>&>(),
          py::arg("kp"), py::arg("ki"), py::arg("kd"),
          doc.PidController.ctor.doc_3args)
      .def(py::init<const MatrixX<double>&, const VectorX<double>&,
               const VectorX<double>&, const VectorX<double>&>(),
          py::arg("state_projection"), py::arg("kp"), py::arg("ki"),
          py::arg("kd"), doc.PidController.ctor.doc_4args)
      .def(py::init<const MatrixX<double>&, const MatrixX<double>&,
               const VectorX<double>&, const VectorX<double>&,
               const VectorX<double>&>(),
          py::arg("state_projection"), py::arg("output_projection"),
          py::arg("kp"), py::arg("ki"), py::arg("kd"),
          doc.PidController.ctor.doc_5args)
      .def("get_Kp_vector", &PidController<double>::get_Kp_vector,
          doc.PidController.get_Kp_vector.doc)
      .def("get_Ki_vector", &PidController<double>::get_Ki_vector,
          doc.PidController.get_Ki_vector.doc)
      .def("get_Kd_vector", &PidController<double>::get_Kd_vector,
          doc.PidController.get_Kd_vector.doc)
      .def("get_input_port_estimated_state",
          &PidController<double>::get_input_port_estimated_state,
          py_rvp::reference_internal,
          doc.PidController.get_input_port_estimated_state.doc)
      .def("get_input_port_desired_state",
          &PidController<double>::get_input_port_desired_state,
          py_rvp::reference_internal,
          doc.PidController.get_input_port_desired_state.doc)
      .def("get_output_port_control",
          &PidController<double>::get_output_port_control,
          py_rvp::reference_internal,
          doc.PidController.get_output_port_control.doc);

  m.def("FittedValueIteration", WrapCallbacks(&FittedValueIteration),
      doc.FittedValueIteration.doc);

  m.def("LinearProgrammingApproximateDynamicProgramming",
      WrapCallbacks(&LinearProgrammingApproximateDynamicProgramming),
      doc.LinearProgrammingApproximateDynamicProgramming.doc);

  m.def(
      "LinearQuadraticRegulator",
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

  m.def(
      "DiscreteTimeLinearQuadraticRegulator",
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

  py::class_<FiniteHorizonLinearQuadraticRegulatorOptions> fhlqr_options(m,
      "FiniteHorizonLinearQuadraticRegulatorOptions",
      doc.FiniteHorizonLinearQuadraticRegulatorOptions.doc);
  fhlqr_options
      .def(py::init<>(),
          doc.FiniteHorizonLinearQuadraticRegulatorOptions.ctor.doc)
      .def_readwrite("Qf", &FiniteHorizonLinearQuadraticRegulatorOptions::Qf,
          doc.FiniteHorizonLinearQuadraticRegulatorOptions.Qf.doc)
      .def_readwrite("N", &FiniteHorizonLinearQuadraticRegulatorOptions::N,
          doc.FiniteHorizonLinearQuadraticRegulatorOptions.N.doc)
      .def_readwrite("input_port_index",
          &FiniteHorizonLinearQuadraticRegulatorOptions::input_port_index,
          doc.FiniteHorizonLinearQuadraticRegulatorOptions.input_port_index.doc)
      .def("__repr__",
          [](const FiniteHorizonLinearQuadraticRegulatorOptions& self) {
            return py::str(
                "FiniteHorizonLinearQuadraticRegulatorOptions("
                "Qf={}, "
                "N={}, "
                "input_port_index={})")
                .format(self.Qf, self.N, self.input_port_index);
          });

  DefReadWriteKeepAlive(&fhlqr_options, "x0",
      &FiniteHorizonLinearQuadraticRegulatorOptions::x0,
      doc.FiniteHorizonLinearQuadraticRegulatorOptions.x0.doc);
  DefReadWriteKeepAlive(&fhlqr_options, "u0",
      &FiniteHorizonLinearQuadraticRegulatorOptions::u0,
      doc.FiniteHorizonLinearQuadraticRegulatorOptions.u0.doc);
  DefReadWriteKeepAlive(&fhlqr_options, "xd",
      &FiniteHorizonLinearQuadraticRegulatorOptions::xd,
      doc.FiniteHorizonLinearQuadraticRegulatorOptions.xd.doc);
  DefReadWriteKeepAlive(&fhlqr_options, "ud",
      &FiniteHorizonLinearQuadraticRegulatorOptions::ud,
      doc.FiniteHorizonLinearQuadraticRegulatorOptions.ud.doc);

  py::class_<FiniteHorizonLinearQuadraticRegulatorResult> fhlqr_result(m,
      "FiniteHorizonLinearQuadraticRegulatorResult",
      doc.FiniteHorizonLinearQuadraticRegulatorResult.doc);
  DefReadUniquePtr(&fhlqr_result, "x0",
      &FiniteHorizonLinearQuadraticRegulatorResult::x0,
      doc.FiniteHorizonLinearQuadraticRegulatorResult.x0.doc);
  DefReadUniquePtr(&fhlqr_result, "u0",
      &FiniteHorizonLinearQuadraticRegulatorResult::u0,
      doc.FiniteHorizonLinearQuadraticRegulatorResult.u0.doc);
  DefReadUniquePtr(&fhlqr_result, "K",
      &FiniteHorizonLinearQuadraticRegulatorResult::K,
      doc.FiniteHorizonLinearQuadraticRegulatorResult.K.doc);
  DefReadUniquePtr(&fhlqr_result, "S",
      &FiniteHorizonLinearQuadraticRegulatorResult::S,
      doc.FiniteHorizonLinearQuadraticRegulatorResult.S.doc);
  DefReadUniquePtr(&fhlqr_result, "k0",
      &FiniteHorizonLinearQuadraticRegulatorResult::k0,
      doc.FiniteHorizonLinearQuadraticRegulatorResult.k0.doc);
  DefReadUniquePtr(&fhlqr_result, "sx",
      &FiniteHorizonLinearQuadraticRegulatorResult::sx,
      doc.FiniteHorizonLinearQuadraticRegulatorResult.sx.doc);
  DefReadUniquePtr(&fhlqr_result, "s0",
      &FiniteHorizonLinearQuadraticRegulatorResult::s0,
      doc.FiniteHorizonLinearQuadraticRegulatorResult.s0.doc);

  m.def("FiniteHorizonLinearQuadraticRegulator",
      &FiniteHorizonLinearQuadraticRegulator, py::arg("system"),
      py::arg("context"), py::arg("t0"), py::arg("tf"), py::arg("Q"),
      py::arg("R"),
      py::arg("options") = FiniteHorizonLinearQuadraticRegulatorOptions(),
      doc.FiniteHorizonLinearQuadraticRegulator.doc);

  m.def("MakeFiniteHorizonLinearQuadraticRegulator",
      &MakeFiniteHorizonLinearQuadraticRegulator, py::arg("system"),
      py::arg("context"), py::arg("t0"), py::arg("tf"), py::arg("Q"),
      py::arg("R"),
      py::arg("options") = FiniteHorizonLinearQuadraticRegulatorOptions(),
      doc.MakeFiniteHorizonLinearQuadraticRegulator.doc);
}

}  // namespace pydrake
}  // namespace drake
