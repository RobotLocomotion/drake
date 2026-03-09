#include "drake/bindings/generated_docstrings/planning.h"
#include "drake/bindings/pydrake/planning/planning_py.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/planning/joint_limits.h"

namespace drake {
namespace pydrake {
namespace internal {

void DefinePlanningJointLimits(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::planning;
  constexpr auto& doc = pydrake_doc_planning.drake.planning;

  using Class = JointLimits;
  constexpr auto& cls_doc = doc.JointLimits;
  py::class_<Class>(m, "JointLimits")
      .def(py::init<>(), cls_doc.ctor.doc)
      .def(py::init<const multibody::MultibodyPlant<double>&, bool, bool,
               bool>(),
          py::arg("plant"), py::arg("require_finite_positions") = false,
          py::arg("require_finite_velocities") = false,
          py::arg("require_finite_accelerations") = false,
          cls_doc.ctor.doc_plant)
      .def(py::init<const multibody::MultibodyPlant<double>&, const DofMask&,
               bool, bool, bool>(),
          py::arg("plant"), py::arg("active_dof"),
          py::arg("require_finite_positions") = false,
          py::arg("require_finite_velocities") = false,
          py::arg("require_finite_accelerations") = false,
          cls_doc.ctor.doc_plant_select)
      .def(py::init<const JointLimits&, const DofMask&, bool, bool, bool>(),
          py::arg("other"), py::arg("active_dof"),
          py::arg("require_finite_positions") = false,
          py::arg("require_finite_velocities") = false,
          py::arg("require_finite_accelerations") = false,
          cls_doc.ctor.doc_copy_select)
      .def(py::init<const Eigen::VectorXd&, const Eigen::VectorXd&,
               const Eigen::VectorXd&, const Eigen::VectorXd&,
               const Eigen::VectorXd&, const Eigen::VectorXd&, bool, bool,
               bool>(),
          py::arg("position_lower"), py::arg("position_upper"),
          py::arg("velocity_lower"), py::arg("velocity_upper"),
          py::arg("acceleration_lower"), py::arg("acceleration_upper"),
          py::arg("require_finite_positions") = false,
          py::arg("require_finite_velocities") = false,
          py::arg("require_finite_accelerations") = false,
          cls_doc.ctor.doc_vectors)
      .def("num_positions", &Class::num_positions, cls_doc.num_positions.doc)
      .def("num_velocities", &Class::num_velocities, cls_doc.num_velocities.doc)
      .def("num_accelerations", &Class::num_accelerations,
          cls_doc.num_accelerations.doc)
      .def("position_lower", &Class::position_lower, cls_doc.position_lower.doc)
      .def("position_upper", &Class::position_upper, cls_doc.position_upper.doc)
      .def("velocity_lower", &Class::velocity_lower, cls_doc.velocity_lower.doc)
      .def("velocity_upper", &Class::velocity_upper, cls_doc.velocity_upper.doc)
      .def("acceleration_lower", &Class::acceleration_lower,
          cls_doc.acceleration_lower.doc)
      .def("acceleration_upper", &Class::acceleration_upper,
          cls_doc.acceleration_upper.doc)
      .def("CheckInPositionLimits", &Class::CheckInPositionLimits,
          py::arg("position"), py::arg("tolerance") = 0.0,
          cls_doc.CheckInPositionLimits.doc)
      .def("CheckInVelocityLimits", &Class::CheckInVelocityLimits,
          py::arg("velocity"), py::arg("tolerance") = 0.0,
          cls_doc.CheckInVelocityLimits.doc)
      .def("CheckInAccelerationLimits", &Class::CheckInAccelerationLimits,
          py::arg("acceleration"), py::arg("tolerance") = 0.0,
          cls_doc.CheckInAccelerationLimits.doc);
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
