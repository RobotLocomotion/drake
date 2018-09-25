#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/multibody/joints/drake_joint.h"
#include "drake/multibody/joints/fixed_axis_one_dof_joint.h"
#include "drake/multibody/joints/prismatic_joint.h"
#include "drake/multibody/joints/revolute_joint.h"

#define D(...) DOC(__VA_ARGS__)

namespace drake {
namespace pydrake {

PYBIND11_MODULE(joints, m) {
  m.doc() = "Joint types supported by Drake.";

  py::class_<DrakeJoint>(m, "DrakeJoint", D(DrakeJoint))
    .def("get_num_positions", &DrakeJoint::get_num_positions,
         D(DrakeJoint, get_num_positions))
    .def("get_name", &DrakeJoint::get_name, D(DrakeJoint, get_name));

  py::class_<PrismaticJoint, DrakeJoint>(m, "PrismaticJoint", D(PrismaticJoint))
    .def(py::init<const std::string&,
                  const Eigen::Isometry3d&,
                  const Eigen::Vector3d&>(),
         py::arg("name"),
         py::arg("transform_to_parent_body"),
         py::arg("translation_axis"), D(DrakeJoint, get_name),
         D(PrismaticJoint, PrismaticJoint));

  py::class_<RevoluteJoint, DrakeJoint>(m, "RevoluteJoint", D(RevoluteJoint))
    .def(py::init<const std::string&,
                  const Eigen::Isometry3d&,
                  const Eigen::Vector3d&>(),
         py::arg("name"),
         py::arg("transform_to_parent_body"),
         py::arg("rotation_axis"),
         D(RevoluteJoint, RevoluteJoint));
}

}  // namespace pydrake
}  // namespace drake
