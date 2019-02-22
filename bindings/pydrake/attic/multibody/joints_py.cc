#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/multibody/joints/drake_joint.h"
#include "drake/multibody/joints/fixed_axis_one_dof_joint.h"
#include "drake/multibody/joints/fixed_joint.h"
#include "drake/multibody/joints/prismatic_joint.h"
#include "drake/multibody/joints/revolute_joint.h"
#include "drake/multibody/joints/roll_pitch_yaw_floating_joint.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(joints, m) {
  constexpr auto& doc = pydrake_doc;

  m.doc() = "Joint types supported by Drake.";

  py::class_<DrakeJoint>(m, "DrakeJoint", doc.DrakeJoint.doc)
      .def("get_num_positions", &DrakeJoint::get_num_positions,
          doc.DrakeJoint.get_num_positions.doc)
      .def("get_name", &DrakeJoint::get_name, doc.DrakeJoint.get_name.doc);

  py::class_<PrismaticJoint, DrakeJoint>(
      m, "PrismaticJoint", doc.PrismaticJoint.doc)
      .def(py::init<const std::string&, const Eigen::Isometry3d&,
               const Eigen::Vector3d&>(),
          py::arg("name"), py::arg("transform_to_parent_body"),
          py::arg("translation_axis"), doc.PrismaticJoint.ctor.doc);

  py::class_<FixedJoint, DrakeJoint>(m, "FixedJoint", doc.FixedJoint.doc)
      .def(py::init<const std::string&, const Eigen::Isometry3d&>(),
          py::arg("name"), py::arg("transform_to_parent_body"),
          doc.FixedJoint.ctor.doc);

  py::class_<RevoluteJoint, DrakeJoint>(
      m, "RevoluteJoint", doc.RevoluteJoint.doc)
      .def(py::init<const std::string&, const Eigen::Isometry3d&,
               const Eigen::Vector3d&>(),
          py::arg("name"), py::arg("transform_to_parent_body"),
          py::arg("rotation_axis"), doc.RevoluteJoint.ctor.doc);

  py::class_<RollPitchYawFloatingJoint, DrakeJoint>(
      m, "RollPitchYawFloatingJoint", doc.RollPitchYawFloatingJoint.doc)
      .def(py::init<const std::string&, const Eigen::Isometry3d&>(),
          py::arg("name"), py::arg("transform_to_parent_body"),
          doc.RollPitchYawFloatingJoint.ctor.doc);
}

}  // namespace pydrake
}  // namespace drake
