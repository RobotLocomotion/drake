#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/multibody/shapes/drake_shapes.h"

#define D(...) DOC(DrakeShapes, __VA_ARGS__)

namespace drake {
namespace pydrake {

PYBIND11_MODULE(shapes, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace DrakeShapes;

  py::module::import("pydrake.util.eigen_geometry");

  m.doc() = "Core geometry and shape types.";

  py::enum_<Shape>(m, "Shape")
    .value("UNKNOWN", Shape::UNKNOWN, D(Shape, UNKNOWN))
    .value("BOX", Shape::BOX, D(Shape, BOX))
    .value("SPHERE", Shape::SPHERE, D(Shape, SPHERE))
    .value("CYLINDER", Shape::CYLINDER, D(Shape, CYLINDER))
    .value("MESH", Shape::MESH, D(Shape, MESH))
    .value("MESH_POINTS", Shape::MESH_POINTS, D(Shape, MESH_POINTS))
    .value("CAPSULE", Shape::CAPSULE, D(Shape, CAPSULE));

  py::class_<Geometry>(m, "Geometry", D(Geometry))
    .def("getShape", &Geometry::getShape, D(Geometry, getShape))
    .def("hasFaces", &Geometry::hasFaces, D(Geometry, hasFaces))
    .def("getFaces", [](const Geometry* self) {
          TrianglesVector triangles;
          self->getFaces(&triangles);
          return triangles;
        }, D(Geometry, getFaces))
    .def("getPoints",
        [](const Geometry* self) {
          Eigen::Matrix3Xd pts(3, 0);
          self->getPoints(pts);
          return pts;
        }, D(Geometry, getPoints))
    .def("getBoundingBoxPoints",
        [](const Geometry* self) {
          Eigen::Matrix3Xd pts(3, 0);
          self->getBoundingBoxPoints(pts);
          return pts;
        }, D(Geometry, getBoundingBoxPoints));

  py::class_<Box, Geometry>(m, "Box", D(Box))
    .def(py::init<const Eigen::Vector3d&>(), py::arg("size"), D(Box, Box))
    .def_readonly("size", &Box::size, D(Box, size));
  py::class_<Sphere, Geometry>(m, "Sphere", D(Sphere))
    .def(py::init<double>(), py::arg("radius"), D(Sphere, Sphere))
    .def_readonly("radius", &Sphere::radius, D(Sphere, radius));
  py::class_<Cylinder, Geometry>(m, "Cylinder", D(Cylinder))
    .def(py::init<double, double>(), py::arg("radius"), py::arg("length"),
         D(Cylinder, Cylinder))
    .def_readonly("radius", &Cylinder::radius, D(Cylinder, radius))
    .def_readonly("length", &Cylinder::length, D(Cylinder, length));
  py::class_<Mesh, Geometry>(m, "Mesh", D(Mesh))
    .def(py::init<const std::string&, const std::string&>(),
         py::arg("uri"), py::arg("resolved_filename"), D(Mesh, Mesh))
    .def_readonly("scale", &Mesh::scale_, D(Mesh, scale))
    .def_readonly("uri", &Mesh::uri_, D(Mesh, uri))
    .def_readonly("resolved_filename", &Mesh::resolved_filename_,
                  D(Mesh, resolved_filename));
  py::class_<MeshPoints, Geometry>(m, "MeshPoints", D(MeshPoints))
    .def(py::init<const Eigen::Matrix3Xd&>(),
         py::arg("points"), D(MeshPoints, MeshPoints));
  py::class_<Capsule, Geometry>(m, "Capsule", D(Capsule))
    .def(py::init<double, double>(), py::arg("radius"), py::arg("length"),
         D(Capsule, Capsule))
    .def_readonly("radius", &Capsule::radius, D(Capsule, radius))
    .def_readonly("length", &Capsule::length, D(Capsule, length));

  py::class_<Element>(m, "Element", D(Element))
    .def(py::init<const Geometry&, const Eigen::Isometry3d&>(),
         py::arg("geometry_in"),
         py::arg("T_element_to_local"), D(Element, Element))
    .def("hasGeometry", &Element::hasGeometry, D(Element, hasGeometry))
    .def("getGeometry", &Element::getGeometry, py_reference_internal,
         D(Element, getGeometry))
    .def("getLocalTransform", [](const Element* self) {
      return self->getLocalTransform().matrix();
    }, D(Element, getLocalTransform));
  py::class_<VisualElement, Element>(m, "VisualElement")
    .def(py::init<const Geometry&, const Eigen::Isometry3d&,
                  const Eigen::Vector4d&>(),
         py::arg("geometry_in"),
         py::arg("T_element_to_local"),
         py::arg("material_in"), D(VisualElement, VisualElement, 2))
    .def("setMaterial", &VisualElement::setMaterial, "Apply an RGBA material.",
         D(VisualElement, setMaterial))
    .def("getMaterial", &VisualElement::getMaterial, "Get an RGBA material.",
         D(VisualElement, getMaterial));
}

}  // namespace pydrake
}  // namespace drake
