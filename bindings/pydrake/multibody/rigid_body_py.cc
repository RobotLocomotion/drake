#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/multibody/joints/prismatic_joint.h"
#include "drake/multibody/joints/revolute_joint.h"
#include "drake/multibody/rigid_body.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(rigid_body, m) {
  m.doc() = "Bindings for the individual RigidBody class.";

  py::module::import("pydrake.multibody.joints");

  py::class_<RigidBody<double> >(m, "RigidBody")
    .def(py::init())
    .def("get_body_index", &RigidBody<double>::get_body_index)
    .def("get_center_of_mass", &RigidBody<double>::get_center_of_mass)
    .def("get_name", &RigidBody<double>::get_name)
    .def("set_name", &RigidBody<double>::set_name)
    .def("get_position_start_index",
         &RigidBody<double>::get_position_start_index)
    .def("get_spatial_inertia", &RigidBody<double>::get_spatial_inertia)
    .def("set_spatial_inertia", &RigidBody<double>::set_spatial_inertia)

    .def("add_joint", &RigidBody<double>::add_joint<DrakeJoint>)
    .def("has_joint", &RigidBody<double>::has_joint)
    .def("getJoint", &RigidBody<double>::getJoint,
         py::return_value_policy::reference_internal)

    .def("get_visual_elements", &RigidBody<double>::get_visual_elements)
    .def("AddVisualElement", &RigidBody<double>::AddVisualElement)
    .def("get_num_collision_elements",
         &RigidBody<double>::get_num_collision_elements)
    .def("get_collision_element_ids",
         &RigidBody<double>::get_collision_element_ids);
}

}  // namespace pydrake
}  // namespace drake
