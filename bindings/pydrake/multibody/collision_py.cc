#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/multibody/collision/element.h"
#include "drake/multibody/rigid_body.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(collision, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::multibody::collision;

  m.doc() = "Drake Collision types.";

  py::module::import("pydrake.multibody.shapes");
  py::module::import("pydrake.multibody.rigid_body");

  py::class_<Element, DrakeShapes::Element>(m, "CollisionElement")
  .def("__init__",
         [](Element& instance, 
          const DrakeShapes::Geometry& geometry_in,
            const Eigen::Matrix4d& T_element_to_local) {
           Eigen::Isometry3d tf;
           tf.matrix() = T_element_to_local;
           new (&instance) Element(geometry_in, tf);
         },
         py::arg("geometry_in"), py::arg("T_element_to_local"))
  .def("set_body", &Element::set_body)
  .def("get_body", &Element::get_body);
}

}  // namespace pydrake
}  // namespace drake
