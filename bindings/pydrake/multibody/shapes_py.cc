#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/multibody/shapes/drake_shapes.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(shapes, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace DrakeShapes;

  py::enum_<Shape>(m, "Shape")
    .value("UNKNOWN", Shape::UNKNOWN)
    .value("BOX", Shape::BOX)
    .value("SPHERE", Shape::SPHERE)
    .value("CYLINDER", Shape::CYLINDER)
    .value("MESH", Shape::MESH)
    .value("MESH_POINTS", Shape::MESH_POINTS)
    .value("CAPSULE", Shape::CAPSULE);

  py::class_<Geometry>(m, "Geometry")
    .def("getShape", &Geometry::getShape)
    .def("hasFaces", &Geometry::hasFaces)
    .def("getFaces", [](const Geometry& self) {
          auto tris = TrianglesVector();
          self.getFaces(&tris);
          return tris;
        })
    .def("getPoints", 
        [](const Geometry& self) {
          auto pts = Eigen::Matrix3Xd(3, 0);
          self.getPoints(pts);
          return pts;
        })
    .def("getBoundingBoxPoints", 
        [](const Geometry& self) {
          auto pts = Eigen::Matrix3Xd(3, 0);
          self.getBoundingBoxPoints(pts);
          return pts;
        });

  py::class_<Box, Geometry>(m, "Box");
  py::class_<Sphere, Geometry>(m, "Sphere")
    .def_readonly("radius", &Sphere::radius);
  py::class_<Cylinder, Geometry>(m, "Cylinder");
  py::class_<Mesh, Geometry>(m, "Mesh");
  py::class_<MeshPoints, Geometry>(m, "MeshPoints");
  py::class_<Capsule, Geometry>(m, "Capsule");

  py::class_<Element>(m, "Element")
    .def("hasGeometry", &Element::hasGeometry)
    .def("getGeometry", &Element::getGeometry, py_reference_internal)
    .def("getLocalTransform", [](const Element& self) {
      auto tf = self.getLocalTransform();
      return tf.matrix();
    });

  py::class_<VisualElement, Element>(m, "VisualElement")
    .def("setMaterial", &VisualElement::setMaterial)
    .def("getMaterial", &VisualElement::getMaterial);
}

} // namespace pydrake
} // namespace drake