#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "drake/bindings/pydrake/common/deprecation_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/manipulation/planner/differential_inverse_kinematics.h"
#include "drake/manipulation/planner/differential_inverse_kinematics_integrator.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(planner, m) {
  using drake::manipulation::planner::DifferentialInverseKinematicsStatus;
  using drake::systems::LeafSystem;

  m.doc() = "Tools for manipulation planning.";
  constexpr auto& doc = pydrake_doc.drake.manipulation.planner;

  py::module::import("pydrake.systems.framework");

  py::enum_<DifferentialInverseKinematicsStatus>(m,
      "DifferentialInverseKinematicsStatus",
      doc.DifferentialInverseKinematicsStatus.doc)
      .value("kSolutionFound",
          DifferentialInverseKinematicsStatus::kSolutionFound,
          doc.DifferentialInverseKinematicsStatus.kSolutionFound.doc)
      .value("kNoSolutionFound",
          DifferentialInverseKinematicsStatus::kNoSolutionFound,
          doc.DifferentialInverseKinematicsStatus.kNoSolutionFound.doc)
      .value("kStuck", DifferentialInverseKinematicsStatus::kStuck,
          doc.DifferentialInverseKinematicsStatus.kStuck.doc);

  {
    using Class = manipulation::planner::DifferentialInverseKinematicsResult;
    constexpr auto& cls_doc = doc.DifferentialInverseKinematicsResult;
    py::class_<Class> cls(m, "DifferentialInverseKinematicsResult",
        doc.DifferentialInverseKinematicsResult.doc);

    // TODO(m-chaturvedi) Add Pybind11 documentation.
    cls  // BR
        .def(ParamInit<Class>())
        .def_readwrite("joint_velocities", &Class::joint_velocities,
            cls_doc.joint_velocities.doc)
        .def_readwrite("status", &Class::status, cls_doc.status.doc);
  }
  {
    using Class =
        manipulation::planner::DifferentialInverseKinematicsParameters;
    constexpr auto& cls_doc = doc.DifferentialInverseKinematicsParameters;

    py::class_<Class> cls(m, "DifferentialInverseKinematicsParameters",
        doc.DifferentialInverseKinematicsParameters.doc);

    cls.def(py::init([](int num_positions, int num_velocities) {
         return Class{num_positions, num_velocities};
       }),
           py::arg("num_positions") = 0, py::arg("num_velocities") = 0,
           cls_doc.ctor.doc)
        .def("get_timestep", &Class::get_timestep, cls_doc.get_timestep.doc)
        .def("set_timestep", &Class::set_timestep, cls_doc.set_timestep.doc)
        .def("get_num_positions", &Class::get_num_positions,
            cls_doc.get_num_positions.doc)
        .def("get_num_velocities", &Class::get_num_velocities,
            cls_doc.get_num_velocities.doc)
        .def("get_nominal_joint_position", &Class::get_nominal_joint_position,
            cls_doc.get_nominal_joint_position.doc)
        .def("set_nominal_joint_position", &Class::set_nominal_joint_position,
            cls_doc.set_nominal_joint_position.doc)
        .def("get_end_effector_velocity_gain",
            &Class::get_end_effector_velocity_gain,
            cls_doc.get_end_effector_velocity_gain.doc)
        .def("set_end_effector_velocity_gain",
            &Class::set_end_effector_velocity_gain,
            cls_doc.set_end_effector_velocity_gain.doc)
        .def("get_unconstrained_degrees_of_freedom_velocity_limit",
            &Class::get_unconstrained_degrees_of_freedom_velocity_limit,
            cls_doc.get_unconstrained_degrees_of_freedom_velocity_limit.doc)
        .def("set_unconstrained_degrees_of_freedom_velocity_limit",
            &Class::set_unconstrained_degrees_of_freedom_velocity_limit,
            cls_doc.set_unconstrained_degrees_of_freedom_velocity_limit.doc)
        .def("get_joint_position_limits", &Class::get_joint_position_limits,
            cls_doc.get_joint_position_limits.doc)
        .def("set_joint_position_limits", &Class::set_joint_position_limits,
            cls_doc.set_joint_position_limits.doc)
        .def("get_joint_velocity_limits", &Class::get_joint_velocity_limits,
            cls_doc.get_joint_velocity_limits.doc)
        .def("set_joint_velocity_limits", &Class::set_joint_velocity_limits,
            cls_doc.set_joint_velocity_limits.doc)
        .def("get_joint_acceleration_limits",
            &Class::get_joint_acceleration_limits,
            cls_doc.get_joint_acceleration_limits.doc)
        .def("set_joint_acceleration_limits",
            &Class::set_joint_acceleration_limits,
            cls_doc.set_joint_acceleration_limits.doc);
  }

  m.def(
      "DoDifferentialInverseKinematics",
      [](const Eigen::VectorXd& q_current, const Eigen::VectorXd& v_current,
          const Eigen::VectorXd& V, const Eigen::MatrixXd& J,
          const manipulation::planner::DifferentialInverseKinematicsParameters&
              parameters) {
        return manipulation::planner::DoDifferentialInverseKinematics(
            q_current, v_current, V, J, parameters);
      },
      py::arg("q_current"), py::arg("v_current"), py::arg("V"), py::arg("J"),
      py::arg("parameters"),
      doc.DoDifferentialInverseKinematics
          .doc_5args_q_current_v_current_V_J_parameters);

  m.def(
      "DoDifferentialInverseKinematics",
      [](const multibody::MultibodyPlant<double>& robot,
          const systems::Context<double>& context,
          const Vector6<double>& V_WE_desired,
          const multibody::Frame<double>& frame_E,
          const manipulation::planner::DifferentialInverseKinematicsParameters&
              parameters) {
        return manipulation::planner::DoDifferentialInverseKinematics(
            robot, context, V_WE_desired, frame_E, parameters);
      },
      py::arg("robot"), py::arg("context"), py::arg("V_WE_desired"),
      py::arg("frame_E"), py::arg("parameters"),
      doc.DoDifferentialInverseKinematics
          .doc_5args_robot_context_V_WE_desired_frame_E_parameters);

  m.def(
      "DoDifferentialInverseKinematics",
      [](const multibody::MultibodyPlant<double>& robot,
          const systems::Context<double>& context,
          const math::RigidTransform<double>& X_WE_desired,
          const multibody::Frame<double>& frame_E,
          const manipulation::planner::DifferentialInverseKinematicsParameters&
              parameters) {
        return manipulation::planner::DoDifferentialInverseKinematics(
            robot, context, X_WE_desired, frame_E, parameters);
      },
      py::arg("robot"), py::arg("context"), py::arg("X_WE_desired"),
      py::arg("frame_E"), py::arg("parameters"),
      doc.DoDifferentialInverseKinematics
          .doc_5args_robot_context_X_WE_desired_frame_E_parameters);
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  m.def("DoDifferentialInverseKinematics",
      WrapDeprecated(
          doc.DoDifferentialInverseKinematics
              .doc_deprecated_deprecated_5args_robot_context_X_WE_desired_frame_E_parameters,
          [](const multibody::MultibodyPlant<double>& robot,
              const systems::Context<double>& context,
              const Isometry3<double>& X_WE_desired,
              const multibody::Frame<double>& frame_E,
              const manipulation::planner::
                  DifferentialInverseKinematicsParameters& parameters) {
            return manipulation::planner::DoDifferentialInverseKinematics(
                robot, context, X_WE_desired, frame_E, parameters);
          }),
      py::arg("robot"), py::arg("context"), py::arg("X_WE_desired"),
      py::arg("frame_E"), py::arg("parameters"),
      doc.DoDifferentialInverseKinematics
          .doc_deprecated_deprecated_5args_robot_context_X_WE_desired_frame_E_parameters);
#pragma GCC diagnostic pop

  {
    using Class =
        manipulation::planner::DifferentialInverseKinematicsIntegrator;
    constexpr auto& cls_doc = doc.DifferentialInverseKinematicsIntegrator;
    py::class_<Class, LeafSystem<double>>(
        m, "DifferentialInverseKinematicsIntegrator", cls_doc.doc)
        .def(py::init<const multibody::MultibodyPlant<double>&,
                 const multibody::Frame<double>&, double,
                 const manipulation::planner::
                     DifferentialInverseKinematicsParameters&,
                 const systems::Context<double>*, bool>(),
            // Keep alive, reference: `self` keeps `robot` alive.
            py::keep_alive<1, 2>(), py::arg("robot"), py::arg("frame_E"),
            py::arg("time_step"), py::arg("parameters"),
            py::arg("robot_context") = nullptr,
            py::arg("log_only_when_result_state_changes") = true,
            cls_doc.ctor.doc)
        .def("SetPositions", &Class::SetPositions, py::arg("context"),
            py::arg("positions"), cls_doc.SetPositions.doc)
        .def("ForwardKinematics", &Class::ForwardKinematics, py::arg("context"),
            cls_doc.ForwardKinematics.doc)
        .def("get_parameters", &Class::get_parameters,
            py_rvp::reference_internal, cls_doc.get_parameters.doc)
        .def("get_mutable_parameters", &Class::get_mutable_parameters,
            py_rvp::reference_internal, cls_doc.get_mutable_parameters.doc);
  }
}

}  // namespace pydrake
}  // namespace drake
