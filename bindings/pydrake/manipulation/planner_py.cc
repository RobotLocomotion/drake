#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/util/drake_optional_pybind.h"
#include "drake/manipulation/planner/differential_inverse_kinematics.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(planner, m) {
  using drake::manipulation::planner::DifferentialInverseKinematicsStatus;
  m.doc() = "Tools for manipulation planning.";
  constexpr auto& doc = pydrake_doc.drake.manipulation.planner;

  py::enum_<DifferentialInverseKinematicsStatus>(
      m, "DifferentialInverseKinematicsStatus",
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
    constexpr auto& class_doc = doc.DifferentialInverseKinematicsResult;
    py::class_<Class> cls(m, "DifferentialInverseKinematicsResult",
                          doc.DifferentialInverseKinematicsResult.doc);

    // TODO(m-chaturvedi) Add Pybind11 documentation.
    cls.def(py::init([](optional<VectorX<double>> joint_velocities,
                        DifferentialInverseKinematicsStatus status) {
              return Class{joint_velocities, status};
            }),
            py::arg("joint_velocities"), py::arg("status"))
        .def_readwrite("joint_velocities", &Class::joint_velocities,
                       class_doc.joint_velocities.doc)
        .def_readwrite("status", &Class::status, class_doc.status.doc);
  }
  {
    using Class =
        manipulation::planner::DifferentialInverseKinematicsParameters;
    constexpr auto& class_doc = doc.DifferentialInverseKinematicsParameters;

    py::class_<Class> cls(m, "DifferentialInverseKinematicsParameters",
                          doc.DifferentialInverseKinematicsParameters.doc);

    cls.def(py::init([](int num_positions, int num_velocities) {
              return Class{num_positions, num_velocities};
            }),
            py::arg("num_positions") = 0, py::arg("num_velocities") = 0,
            class_doc.ctor.doc)
        .def("get_timestep", &Class::get_timestep, class_doc.get_timestep.doc)
        .def("set_timestep", &Class::set_timestep, class_doc.set_timestep.doc)
        .def("get_num_positions", &Class::get_num_positions,
             class_doc.get_num_positions.doc)
        .def("get_num_velocities", &Class::get_num_velocities,
             class_doc.get_num_velocities.doc)
        .def("get_nominal_joint_position", &Class::get_nominal_joint_position,
             class_doc.get_nominal_joint_position.doc)
        .def("set_nominal_joint_position", &Class::set_nominal_joint_position,
             class_doc.set_nominal_joint_position.doc)
        .def("get_end_effector_velocity_gain",
             &Class::get_end_effector_velocity_gain,
             class_doc.get_end_effector_velocity_gain.doc)
        .def("set_end_effector_velocity_gain",
             &Class::set_end_effector_velocity_gain,
             class_doc.set_end_effector_velocity_gain.doc)
        .def("get_unconstrained_degrees_of_freedom_velocity_limit",
             &Class::get_unconstrained_degrees_of_freedom_velocity_limit,
             class_doc.get_unconstrained_degrees_of_freedom_velocity_limit.doc)
        .def("set_unconstrained_degrees_of_freedom_velocity_limit",
             &Class::set_unconstrained_degrees_of_freedom_velocity_limit,
             class_doc.set_unconstrained_degrees_of_freedom_velocity_limit.doc)
        .def("get_joint_position_limits", &Class::get_joint_position_limits,
             class_doc.get_joint_position_limits.doc)
        .def("set_joint_position_limits", &Class::set_joint_position_limits,
             class_doc.set_joint_position_limits.doc)
        .def("get_joint_velocity_limits", &Class::get_joint_velocity_limits,
             class_doc.get_joint_velocity_limits.doc)
        .def("set_joint_velocity_limits", &Class::set_joint_velocity_limits,
             class_doc.set_joint_velocity_limits.doc)
        .def("get_joint_acceleration_limits",
             &Class::get_joint_acceleration_limits,
             class_doc.get_joint_acceleration_limits.doc)
        .def("set_joint_acceleration_limits",
             &Class::set_joint_acceleration_limits,
             class_doc.set_joint_acceleration_limits.doc);
  }

  m.def("DoDifferentialInverseKinematics",
        [](const Eigen::VectorXd& q_current, const Eigen::VectorXd& v_current,
           const Eigen::VectorXd& V, const Eigen::MatrixXd& J,
           const manipulation::planner::DifferentialInverseKinematicsParameters&
               parameters) {
          return manipulation::planner::DoDifferentialInverseKinematics(
              q_current, v_current, V, J, parameters);
        },
        py::arg("q_current"), py::arg("v_current"), py::arg("V"), py::arg("J"),
        py::arg("parameters"), doc.DoDifferentialInverseKinematics.doc);

  m.def("DoDifferentialInverseKinematics",
        [](const multibody::multibody_plant::MultibodyPlant<double>& robot,
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
        doc.DoDifferentialInverseKinematics.doc_4);

  m.def("DoDifferentialInverseKinematics",
        [](const multibody::multibody_plant::MultibodyPlant<double>& robot,
           const systems::Context<double>& context,
           const Isometry3<double>& X_WE_desired,
           const multibody::Frame<double>& frame_E,
           const manipulation::planner::DifferentialInverseKinematicsParameters&
               parameters) {
          return manipulation::planner::DoDifferentialInverseKinematics(
              robot, context, X_WE_desired, frame_E, parameters);
        },
        py::arg("robot"), py::arg("context"), py::arg("X_WE_desired"),
        py::arg("frame_E"), py::arg("parameters"),
        doc.DoDifferentialInverseKinematics.doc_5);
}

}  // namespace pydrake
}  // namespace drake
