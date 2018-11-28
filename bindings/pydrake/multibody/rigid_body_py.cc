#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/multibody/joints/prismatic_joint.h"
#include "drake/multibody/joints/revolute_joint.h"
#include "drake/multibody/rigid_body.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(rigid_body, m) {
  constexpr auto& doc = pydrake_doc;

  m.doc() = "Bindings for the individual RigidBody class.";

  py::module::import("pydrake.multibody.joints");

  py::class_<RigidBody<double> >(m, "RigidBody", doc.RigidBody.doc)
      .def(py::init(), doc.RigidBody.ctor.doc)
      .def("get_body_index", &RigidBody<double>::get_body_index,
          doc.RigidBody.get_body_index.doc)
      .def("get_center_of_mass", &RigidBody<double>::get_center_of_mass,
          doc.RigidBody.get_center_of_mass.doc)
      .def("get_name", &RigidBody<double>::get_name, doc.RigidBody.get_name.doc)
      .def("set_name", &RigidBody<double>::set_name, doc.RigidBody.set_name.doc)
      .def("get_position_start_index",
          &RigidBody<double>::get_position_start_index,
          doc.RigidBody.get_position_start_index.doc)
      .def("get_spatial_inertia", &RigidBody<double>::get_spatial_inertia,
          doc.RigidBody.get_spatial_inertia.doc)
      .def("set_spatial_inertia", &RigidBody<double>::set_spatial_inertia,
          doc.RigidBody.set_spatial_inertia.doc)

      .def("add_joint", &RigidBody<double>::add_joint<DrakeJoint>,
          doc.RigidBody.add_joint.doc)
      .def("has_joint", &RigidBody<double>::has_joint,
          doc.RigidBody.has_joint.doc)
      .def("getJoint", &RigidBody<double>::getJoint,
          py::return_value_policy::reference_internal,
          doc.RigidBody.getJoint.doc)

      .def("get_visual_elements", &RigidBody<double>::get_visual_elements,
          doc.RigidBody.get_visual_elements.doc)
      .def("AddVisualElement", &RigidBody<double>::AddVisualElement,
          doc.RigidBody.AddVisualElement.doc)
      .def("get_num_collision_elements",
          &RigidBody<double>::get_num_collision_elements,
          doc.RigidBody.get_num_collision_elements.doc)
      .def("get_collision_element_ids",
          &RigidBody<double>::get_collision_element_ids,
          doc.RigidBody.get_collision_element_ids.doc);
}

}  // namespace pydrake
}  // namespace drake
