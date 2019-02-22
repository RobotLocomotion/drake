#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/multibody/shapes/drake_shapes.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(shapes, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace DrakeShapes;
  constexpr auto& doc = pydrake_doc.DrakeShapes;

  py::module::import("pydrake.common.eigen_geometry");

  m.doc() = "Core geometry and shape types.";

  py::enum_<Shape>(m, "Shape")
      .value("UNKNOWN", Shape::UNKNOWN, doc.Shape.UNKNOWN.doc)
      .value("BOX", Shape::BOX, doc.Shape.BOX.doc)
      .value("SPHERE", Shape::SPHERE, doc.Shape.SPHERE.doc)
      .value("CYLINDER", Shape::CYLINDER, doc.Shape.CYLINDER.doc)
      .value("MESH", Shape::MESH, doc.Shape.MESH.doc)
      .value("MESH_POINTS", Shape::MESH_POINTS, doc.Shape.MESH_POINTS.doc)
      .value("CAPSULE", Shape::CAPSULE, doc.Shape.CAPSULE.doc);

  py::class_<Geometry>(m, "Geometry", doc.Geometry.doc)
      .def("getShape", &Geometry::getShape, doc.Geometry.getShape.doc)
      .def("hasFaces", &Geometry::hasFaces, doc.Geometry.hasFaces.doc)
      .def("getFaces",
          [](const Geometry* self) {
            TrianglesVector triangles;
            self->getFaces(&triangles);
            return triangles;
          },
          doc.Geometry.getFaces.doc)
      .def("getPoints",
          [](const Geometry* self) {
            Eigen::Matrix3Xd pts(3, 0);
            self->getPoints(pts);
            return pts;
          },
          doc.Geometry.getPoints.doc)
      .def("getBoundingBoxPoints",
          [](const Geometry* self) {
            Eigen::Matrix3Xd pts(3, 0);
            self->getBoundingBoxPoints(pts);
            return pts;
          },
          doc.Geometry.getBoundingBoxPoints.doc);

  py::class_<Box, Geometry>(m, "Box", doc.Box.doc)
      .def(
          py::init<const Eigen::Vector3d&>(), py::arg("size"), doc.Box.ctor.doc)
      .def_readonly("size", &Box::size, doc.Box.size.doc);
  py::class_<Sphere, Geometry>(m, "Sphere", doc.Sphere.doc)
      .def(py::init<double>(), py::arg("radius"), doc.Sphere.ctor.doc)
      .def_readonly("radius", &Sphere::radius, doc.Sphere.radius.doc);
  py::class_<Cylinder, Geometry>(m, "Cylinder", doc.Cylinder.doc)
      .def(py::init<double, double>(), py::arg("radius"), py::arg("length"),
          doc.Cylinder.ctor.doc)
      .def_readonly("radius", &Cylinder::radius, doc.Cylinder.radius.doc)
      .def_readonly("length", &Cylinder::length, doc.Cylinder.length.doc);
  py::class_<Mesh, Geometry>(m, "Mesh", doc.Mesh.doc)
      .def(py::init<const std::string&, const std::string&>(), py::arg("uri"),
          py::arg("resolved_filename"), doc.Mesh.ctor.doc)
      .def_readonly("scale", &Mesh::scale_, doc.Mesh.scale_.doc)
      .def_readonly("uri", &Mesh::uri_, doc.Mesh.uri_.doc)
      .def_readonly("resolved_filename", &Mesh::resolved_filename_,
          doc.Mesh.resolved_filename_.doc);
  py::class_<MeshPoints, Geometry>(m, "MeshPoints", doc.MeshPoints.doc)
      .def(py::init<const Eigen::Matrix3Xd&>(), py::arg("points"),
          doc.MeshPoints.ctor.doc);
  py::class_<Capsule, Geometry>(m, "Capsule", doc.Capsule.doc)
      .def(py::init<double, double>(), py::arg("radius"), py::arg("length"),
          doc.Capsule.ctor.doc)
      .def_readonly("radius", &Capsule::radius, doc.Capsule.radius.doc)
      .def_readonly("length", &Capsule::length, doc.Capsule.length.doc);

  py::class_<Element>(m, "Element", doc.Element.doc)
      .def(py::init<const Geometry&, const Eigen::Isometry3d&>(),
          py::arg("geometry_in"), py::arg("T_element_to_local"))
      .def("hasGeometry", &Element::hasGeometry, doc.Element.hasGeometry.doc)
      .def("getGeometry", &Element::getGeometry, py_reference_internal,
          doc.Element.getGeometry.doc)
      .def("getLocalTransform",
          [](const Element* self) {
            return self->getLocalTransform().matrix();
          },
          doc.Element.getLocalTransform.doc);
  py::class_<VisualElement, Element>(m, "VisualElement")
      .def(py::init<const Geometry&, const Eigen::Isometry3d&,
               const Eigen::Vector4d&, const std::string&>(),
          py::arg("geometry_in"), py::arg("T_element_to_local"),
          py::arg("material_in"), py::arg("name") = "",
          doc.VisualElement.ctor.doc)
      .def("setMaterial", &VisualElement::setMaterial,
          "Apply an RGBA material.", doc.VisualElement.setMaterial.doc)
      .def("getMaterial", &VisualElement::getMaterial, "Get an RGBA material.",
          doc.VisualElement.getMaterial.doc);
}

}  // namespace pydrake
}  // namespace drake
