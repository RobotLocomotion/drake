#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/multibody/collision/element.h"
#include "drake/multibody/rigid_body.h"

#define D(...) DOC(drake, multibody, collision, __VA_ARGS__)

namespace drake {
namespace pydrake {

PYBIND11_MODULE(collision, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::multibody::collision;

  m.doc() = "Drake Collision types.";

  py::module::import("pydrake.multibody.shapes");
  py::module::import("pydrake.multibody.rigid_body");

  py::class_<Element, DrakeShapes::Element>(m, "CollisionElement")
  .def(py::init<const DrakeShapes::Geometry&, const Eigen::Isometry3d&>(),
       py::arg("geometry_in"), py::arg("T_element_to_local"),
       D(Element, Element))
  .def("set_body", &Element::set_body, D(Element, set_body))
  .def("get_body", &Element::get_body, D(Element, get_body));
}

}  // namespace pydrake
}  // namespace drake
