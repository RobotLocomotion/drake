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
  py::class_<RigidBody<double> >(m, "RigidBody")
    .def(py::init())
    .def("get_name", &RigidBody<double>::get_name)
    .def("set_name", &RigidBody<double>::set_name)
    .def("get_body_index", &RigidBody<double>::get_body_index)
    .def("get_spatial_inertia", &RigidBody<double>::get_spatial_inertia)
    .def("set_spatial_inertia", &RigidBody<double>::set_spatial_inertia)

    .def("get_visual_elements", &RigidBody<double>::get_visual_elements)
    .def("add_prismatic_joint", &RigidBody<double>::add_joint<PrismaticJoint>)
    .def("add_revolute_joint", &RigidBody<double>::add_joint<RevoluteJoint>)
    .def("AddVisualElement", &RigidBody<double>::AddVisualElement);
}

}  // namespace pydrake
}  // namespace drake
