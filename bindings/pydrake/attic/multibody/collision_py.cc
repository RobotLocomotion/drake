#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/multibody/collision/element.h"
#include "drake/multibody/rigid_body.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(collision, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::multibody::collision;
  constexpr auto& doc = pydrake_doc.drake.multibody.collision;

  m.doc() = "Drake Collision types.";

  py::module::import("pydrake.attic.multibody.shapes");
  py::module::import("pydrake.attic.multibody.rigid_body");

  py::class_<Element, DrakeShapes::Element>(
      m, "CollisionElement", doc.Element.doc)
      .def(py::init<const DrakeShapes::Geometry&, const Eigen::Isometry3d&>(),
          py::arg("geometry_in"), py::arg("T_element_to_local"),
          doc.Element.ctor.doc_2args_geometry_T_element_to_local)
      .def("set_body", &Element::set_body, doc.Element.set_body.doc)
      .def("get_body", &Element::get_body, doc.Element.get_body.doc);

  py::class_<PointPair<double>>(m, "PointPair")
      .def_readonly(
          "elementA", &PointPair<double>::elementA, py_reference_internal)
      .def_readonly(
          "elementB", &PointPair<double>::elementB, py_reference_internal)
      .def_readonly("idA", &PointPair<double>::idA)
      .def_readonly("idB", &PointPair<double>::idB)
      .def_readonly("ptA", &PointPair<double>::ptA)
      .def_readonly("ptB", &PointPair<double>::ptB)
      .def_readonly("normal", &PointPair<double>::normal)
      .def_readonly("distance", &PointPair<double>::distance);
}

}  // namespace pydrake
}  // namespace drake
