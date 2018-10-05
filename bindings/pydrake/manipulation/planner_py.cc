#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/util/drake_optional_pybind.h"
#include "drake/manipulation/planner/differential_inverse_kinematics.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(planner, m) {
  using drake::manipulation::planner::DifferentialInverseKinematicsStatus;
  m.doc() = "Tools for manipulation planning.";

  py::enum_<DifferentialInverseKinematicsStatus>(
      m, "DifferentialInverseKinematicsStatus")
      .value("kSolutionFound",
             DifferentialInverseKinematicsStatus::kSolutionFound)
      .value("kNoSolutionFound",
             DifferentialInverseKinematicsStatus::kNoSolutionFound)
      .value("kStuck", DifferentialInverseKinematicsStatus::kStuck);

  {
    using Class = manipulation::planner::DifferentialInverseKinematicsResult;
    py::class_<Class> cls(m, "DifferentialInverseKinematicsResult");

    cls
        .def(py::init([](optional<VectorX<double>> joint_velocities,
                          DifferentialInverseKinematicsStatus status) {
           return Class{joint_velocities, status};
         }), py::arg("joint_velocities"), py::arg("status"))
        .def_readwrite("joint_velocities", &Class::joint_velocities)
        .def_readwrite("status", &Class::status);
  }
  {
    using Class =
        manipulation::planner::DifferentialInverseKinematicsParameters;
    py::class_<Class> cls(m, "DifferentialInverseKinematicsParameters");

    cls.def(py::init([](int num_positions, int num_velocities) {
              return Class{num_positions, num_velocities};
            }),
            py::arg("num_positions") = 0, py::arg("num_velocities") = 0)
        .def("get_timestep", &Class::get_timestep)
        .def("set_timestep", &Class::set_timestep)
        .def("get_num_positions", &Class::get_num_positions)
        .def("get_num_velocities", &Class::get_num_velocities)
        .def("get_nominal_joint_position", &Class::get_nominal_joint_position)
        .def("set_nominal_joint_position", &Class::set_nominal_joint_position)
        .def("get_end_effector_velocity_gain",
             &Class::get_end_effector_velocity_gain)
        .def("set_end_effector_velocity_gain",
             &Class::set_end_effector_velocity_gain)
        .def("get_unconstrained_degrees_of_freedom_velocity_limit",
             &Class::get_unconstrained_degrees_of_freedom_velocity_limit)
        .def("set_unconstrained_degrees_of_freedom_velocity_limit",
             &Class::set_unconstrained_degrees_of_freedom_velocity_limit)
        .def("get_joint_position_limits", &Class::get_joint_position_limits)
        .def("set_joint_position_limits", &Class::set_joint_position_limits)
        .def("get_joint_velocity_limits", &Class::get_joint_velocity_limits)
        .def("set_joint_velocity_limits", &Class::set_joint_velocity_limits)
        .def("get_joint_acceleration_limits",
             &Class::get_joint_acceleration_limits)
        .def("set_joint_acceleration_limits",
             &Class::set_joint_acceleration_limits);
  }

  m.def("DoDifferentialInverseKinematics",
        [](const Eigen::VectorXd& q_current, const Eigen::VectorXd& v_current,
           const Eigen::VectorXd& V, const Eigen::MatrixXd& J,
           const manipulation::planner::DifferentialInverseKinematicsParameters&
               parameters) {
          return manipulation::planner::DoDifferentialInverseKinematics(
              q_current, v_current, V, J, parameters);
        },
        py::arg("q_current"), py::arg("v_current"),
        py::arg("V"), py::arg("J"), py::arg("parameters"));
}

}  // namespace pydrake
}  // namespace drake
