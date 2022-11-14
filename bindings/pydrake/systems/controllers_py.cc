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
#include "drake/systems/controllers/joint_stiffness_controller.h"
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

  {
    using Class = DynamicProgrammingOptions;
    constexpr auto& cls_doc = doc.DynamicProgrammingOptions;
    py::class_<DynamicProgrammingOptions> cls(
        m, "DynamicProgrammingOptions", cls_doc.doc);
    {
      using Nested = Class::PeriodicBoundaryCondition;
      constexpr auto& nested_doc = cls_doc.PeriodicBoundaryCondition;
      py::class_<Nested> nested(
          cls, "PeriodicBoundaryCondition", nested_doc.doc);
      nested  // BR
          .def(py::init<int, double, double>(), py::arg("state_index"),
              py::arg("low"), py::arg("high"), nested_doc.ctor.doc)
          .def_readwrite(
              "state_index", &Nested::state_index, nested_doc.state_index.doc)
          .def_readwrite("low", &Nested::low, nested_doc.low.doc)
          .def_readwrite("high", &Nested::high, nested_doc.high.doc);
      // TODO(eric.cousineau): Deprecate module-scope alias.
      m.attr("PeriodicBoundaryCondition") = nested;
    }
    cls  // BR
        .def(py::init<>(), cls_doc.ctor.doc)
        .def_readwrite("discount_factor", &Class::discount_factor,
            cls_doc.discount_factor.doc)
        .def_readwrite("periodic_boundary_conditions",
            &Class::periodic_boundary_conditions,
            cls_doc.periodic_boundary_conditions.doc)
        .def_readwrite("convergence_tol", &Class::convergence_tol,
            cls_doc.convergence_tol.doc)
        .def_readwrite("visualization_callback", &Class::visualization_callback,
            cls_doc.visualization_callback.doc)
        .def_readwrite("input_port_index", &Class::input_port_index,
            cls_doc.input_port_index.doc)
        .def_readwrite("assume_non_continuous_states_are_fixed",
            &Class::assume_non_continuous_states_are_fixed,
            cls_doc.assume_non_continuous_states_are_fixed.doc);
  }

  // TODO(russt): Bind all default scalars.

  {
    using Class = InverseDynamics<double>;
    constexpr auto& cls_doc = doc.InverseDynamics;
    py::class_<Class, LeafSystem<double>> cls(
        m, "InverseDynamics", cls_doc.doc);
    {
      using Nested = Class::InverseDynamicsMode;
      constexpr auto& nested_doc = cls_doc.InverseDynamicsMode;
      py::enum_<Nested>(cls, "InverseDynamicsMode")
          .value("kInverseDynamics", Nested::kInverseDynamics, nested_doc.doc)
          .value("kGravityCompensation", Nested::kGravityCompensation,
              nested_doc.kGravityCompensation.doc)
          .export_values();
    }
    cls  // BR
        .def(py::init<const MultibodyPlant<double>*,
                 Class::InverseDynamicsMode>(),
            py::arg("plant"), py::arg("mode") = Class::kInverseDynamics,
            cls_doc.ctor.doc)
        .def("is_pure_gravity_compensation",
            &Class::is_pure_gravity_compensation,
            cls_doc.is_pure_gravity_compensation.doc);
  }

  // TODO(eric.cousineau): Expose multiple inheritance from
  // StateFeedbackControllerInterface once #9243 is resolved.
  {
    using Class = InverseDynamicsController<double>;
    constexpr auto& cls_doc = doc.InverseDynamicsController;
    py::class_<Class, Diagram<double>>(
        m, "InverseDynamicsController", cls_doc.doc)
        .def(py::init<const MultibodyPlant<double>&, const VectorX<double>&,
                 const VectorX<double>&, const VectorX<double>&, bool>(),
            py::arg("robot"), py::arg("kp"), py::arg("ki"), py::arg("kd"),
            py::arg("has_reference_acceleration"),
            // Keep alive, reference: `self` keeps `robot` alive.
            py::keep_alive<1, 2>(), cls_doc.ctor.doc)
        .def("set_integral_value", &Class::set_integral_value,
            cls_doc.set_integral_value.doc)
        .def("get_input_port_desired_acceleration",
            &InverseDynamicsController<
                double>::get_input_port_desired_acceleration,
            py_rvp::reference_internal,
            cls_doc.get_input_port_desired_acceleration.doc)
        .def("get_input_port_estimated_state",
            &Class::get_input_port_estimated_state, py_rvp::reference_internal,
            cls_doc.get_input_port_estimated_state.doc)
        .def("get_input_port_desired_state",
            &Class::get_input_port_desired_state, py_rvp::reference_internal,
            cls_doc.get_input_port_desired_state.doc)
        .def("get_output_port_control", &Class::get_output_port_control,
            py_rvp::reference_internal, cls_doc.get_output_port_control.doc)
        .def("get_multibody_plant_for_control",
            &Class::get_multibody_plant_for_control, py_rvp::reference_internal,
            cls_doc.get_multibody_plant_for_control.doc);
  }

  {
    using Class = JointStiffnessController<double>;
    constexpr auto& cls_doc = doc.JointStiffnessController;
    py::class_<Class, LeafSystem<double>>(
        m, "JointStiffnessController", cls_doc.doc)
        .def(py::init<const MultibodyPlant<double>&,
                 const Eigen::Ref<const Eigen::VectorXd>&,
                 const Eigen::Ref<const Eigen::VectorXd>&>(),
            py::arg("plant"), py::arg("kp"), py::arg("kd"),
            // Keep alive, reference: `self` keeps `robot` alive.
            py::keep_alive<1, 2>(), cls_doc.ctor.doc)
        .def("get_input_port_estimated_state",
            &Class::get_input_port_estimated_state, py_rvp::reference_internal,
            cls_doc.get_input_port_estimated_state.doc)
        .def("get_input_port_desired_state",
            &Class::get_input_port_desired_state, py_rvp::reference_internal,
            cls_doc.get_input_port_desired_state.doc)
        .def("get_output_port_generalized_force",
            &Class::get_output_port_generalized_force,
            py_rvp::reference_internal,
            cls_doc.get_output_port_generalized_force.doc)
        .def("get_multibody_plant", &Class::get_multibody_plant,
            py_rvp::reference_internal, cls_doc.get_multibody_plant.doc);
  }

  {
    using Class = PidControlledSystem<double>;
    constexpr auto& cls_doc = doc.PidControlledSystem;
    py::class_<Class, Diagram<double>>(m, "PidControlledSystem", cls_doc.doc)
        .def(py::init<std::unique_ptr<System<double>>, double, double, double,
                 int, int>(),
            py::arg("plant"), py::arg("kp"), py::arg("ki"), py::arg("kd"),
            py::arg("state_output_port_index") = 0,
            py::arg("plant_input_port_index") = 0,
            // Keep alive, ownership: `plant` keeps `self` alive.
            py::keep_alive<2, 1>(), cls_doc.ctor.doc_6args_double_gains)
        .def(py::init<std::unique_ptr<System<double>>, const VectorX<double>&,
                 const VectorX<double>&, const VectorX<double>&, int, int>(),
            py::arg("plant"), py::arg("kp"), py::arg("ki"), py::arg("kd"),
            py::arg("state_output_port_index") = 0,
            py::arg("plant_input_port_index") = 0,
            // Keep alive, ownership: `plant` keeps `self` alive.
            py::keep_alive<2, 1>(), cls_doc.ctor.doc_6args_vector_gains)
        .def(py::init<std::unique_ptr<System<double>>, const MatrixX<double>&,
                 double, double, double, int, int>(),
            py::arg("plant"), py::arg("feedback_selector"), py::arg("kp"),
            py::arg("ki"), py::arg("kd"),
            py::arg("state_output_port_index") = 0,
            py::arg("plant_input_port_index") = 0,
            // Keep alive, ownership: `plant` keeps `self` alive.
            py::keep_alive<2, 1>(), cls_doc.ctor.doc_7args_double_gains)
        .def(py::init<std::unique_ptr<System<double>>, const MatrixX<double>&,
                 const VectorX<double>&, const VectorX<double>&,
                 const VectorX<double>&, int, int>(),
            py::arg("plant"), py::arg("feedback_selector"), py::arg("kp"),
            py::arg("ki"), py::arg("kd"),
            py::arg("state_output_port_index") = 0,
            py::arg("plant_input_port_index") = 0,
            // Keep alive, ownership: `plant` keeps `self` alive.
            py::keep_alive<2, 1>(), cls_doc.ctor.doc_7args_vector_gains)
        .def("get_control_input_port", &Class::get_control_input_port,
            py_rvp::reference_internal, cls_doc.get_control_input_port.doc)
        .def("get_state_input_port", &Class::get_state_input_port,
            py_rvp::reference_internal, cls_doc.get_state_input_port.doc)
        .def("get_state_output_port", &Class::get_state_output_port,
            py_rvp::reference_internal, cls_doc.get_state_output_port.doc);
  }

  // TODO(eric.cousineau): Expose multiple inheritance from
  // StateFeedbackControllerInterface once #9243 is resolved.
  {
    using Class = PidController<double>;
    constexpr auto& cls_doc = doc.PidController;
    py::class_<Class, LeafSystem<double>>(m, "PidController", cls_doc.doc)
        .def(py::init<const VectorX<double>&, const VectorX<double>&,
                 const VectorX<double>&>(),
            py::arg("kp"), py::arg("ki"), py::arg("kd"), cls_doc.ctor.doc_3args)
        .def(py::init<const MatrixX<double>&, const VectorX<double>&,
                 const VectorX<double>&, const VectorX<double>&>(),
            py::arg("state_projection"), py::arg("kp"), py::arg("ki"),
            py::arg("kd"), cls_doc.ctor.doc_4args)
        .def(py::init<const MatrixX<double>&, const MatrixX<double>&,
                 const VectorX<double>&, const VectorX<double>&,
                 const VectorX<double>&>(),
            py::arg("state_projection"), py::arg("output_projection"),
            py::arg("kp"), py::arg("ki"), py::arg("kd"), cls_doc.ctor.doc_5args)
        .def("get_Kp_vector", &Class::get_Kp_vector, cls_doc.get_Kp_vector.doc)
        .def("get_Ki_vector", &Class::get_Ki_vector, cls_doc.get_Ki_vector.doc)
        .def("get_Kd_vector", &Class::get_Kd_vector, cls_doc.get_Kd_vector.doc)
        .def("get_input_port_estimated_state",
            &Class::get_input_port_estimated_state, py_rvp::reference_internal,
            cls_doc.get_input_port_estimated_state.doc)
        .def("get_input_port_desired_state",
            &Class::get_input_port_desired_state, py_rvp::reference_internal,
            cls_doc.get_input_port_desired_state.doc)
        .def("get_output_port_control", &Class::get_output_port_control,
            py_rvp::reference_internal, cls_doc.get_output_port_control.doc);
  }

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
          const Eigen::Ref<const Eigen::MatrixXd>& N,
          const Eigen::Ref<const Eigen::MatrixXd>& F) {
        auto result = LinearQuadraticRegulator(A, B, Q, R, N, F);
        return std::make_pair(result.K, result.S);
      },
      py::arg("A"), py::arg("B"), py::arg("Q"), py::arg("R"),
      py::arg("N") = Eigen::Matrix<double, 0, 0>::Zero(),
      py::arg("F") = Eigen::Matrix<double, 0, 0>::Zero(),
      doc.LinearQuadraticRegulator.doc_AB);

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
      doc.LinearQuadraticRegulator.doc_system);

  m.def("LinearQuadraticRegulator",
      py::overload_cast<const systems::System<double>&,
          const systems::Context<double>&,
          const Eigen::Ref<const Eigen::MatrixXd>&,
          const Eigen::Ref<const Eigen::MatrixXd>&,
          const Eigen::Ref<const Eigen::MatrixXd>&, int>(
          &LinearQuadraticRegulator),
      py::arg("system"), py::arg("context"), py::arg("Q"), py::arg("R"),
      py::arg("N") = Eigen::Matrix<double, 0, 0>::Zero(),
      py::arg("input_port_index") = 0,
      doc.LinearQuadraticRegulator.doc_linearize_at_context);

  {
    using Class = FiniteHorizonLinearQuadraticRegulatorOptions;
    constexpr auto& cls_doc = doc.FiniteHorizonLinearQuadraticRegulatorOptions;
    py::class_<Class> cls(
        m, "FiniteHorizonLinearQuadraticRegulatorOptions", cls_doc.doc);
    cls  // BR
        .def(py::init<>(), cls_doc.ctor.doc)
        .def_readwrite("Qf", &Class::Qf, cls_doc.Qf.doc)
        .def_readwrite("N", &Class::N, cls_doc.N.doc)
        .def_readwrite("input_port_index", &Class::input_port_index,
            cls_doc.input_port_index.doc)
        .def_readwrite("use_square_root_method", &Class::use_square_root_method,
            cls_doc.use_square_root_method.doc)
        .def("__repr__", [](const Class& self) {
          return py::str(
              "FiniteHorizonLinearQuadraticRegulatorOptions("
              "Qf={}, "
              "N={}, "
              "input_port_index={}, "
              "use_square_root_method={})")
              .format(self.Qf, self.N, self.input_port_index,
                  self.use_square_root_method);
        });
    DefReadWriteKeepAlive(&cls, "x0", &Class::x0, cls_doc.x0.doc);
    DefReadWriteKeepAlive(&cls, "u0", &Class::u0, cls_doc.u0.doc);
    DefReadWriteKeepAlive(&cls, "xd", &Class::xd, cls_doc.xd.doc);
    DefReadWriteKeepAlive(&cls, "ud", &Class::ud, cls_doc.ud.doc);
  }

  {
    using Class = FiniteHorizonLinearQuadraticRegulatorResult;
    constexpr auto& cls_doc = doc.FiniteHorizonLinearQuadraticRegulatorResult;
    py::class_<Class> cls(
        m, "FiniteHorizonLinearQuadraticRegulatorResult", cls_doc.doc);
    DefReadUniquePtr(&cls, "x0", &Class::x0, cls_doc.x0.doc);
    DefReadUniquePtr(&cls, "u0", &Class::u0, cls_doc.u0.doc);
    DefReadUniquePtr(&cls, "K", &Class::K, cls_doc.K.doc);
    DefReadUniquePtr(&cls, "S", &Class::S, cls_doc.S.doc);
    DefReadUniquePtr(&cls, "k0", &Class::k0, cls_doc.k0.doc);
    DefReadUniquePtr(&cls, "sx", &Class::sx, cls_doc.sx.doc);
    DefReadUniquePtr(&cls, "s0", &Class::s0, cls_doc.s0.doc);
  }

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
