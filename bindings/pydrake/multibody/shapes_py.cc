#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/multibody/shapes/drake_shapes.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(shapes, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace DrakeShapes;

  m.doc() = "Core geometry and shape types.";

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
    .def("getFaces", [](const Geometry* self) {
          TrianglesVector triangles;
          self->getFaces(&triangles);
          return triangles;
        })
    .def("getPoints",
        [](const Geometry* self) {
          Eigen::Matrix3Xd pts(3, 0);
          self->getPoints(pts);
          return pts;
        })
    .def("getBoundingBoxPoints",
        [](const Geometry* self) {
          Eigen::Matrix3Xd pts(3, 0);
          self->getBoundingBoxPoints(pts);
          return pts;
        });

  py::class_<Box, Geometry>(m, "Box")
    .def(py::init<const Eigen::Vector3d&>(), py::arg("size"))
    .def_readonly("size", &Box::size);
  py::class_<Sphere, Geometry>(m, "Sphere")
    .def(py::init<double>(), py::arg("radius"))
    .def_readonly("radius", &Sphere::radius);
  py::class_<Cylinder, Geometry>(m, "Cylinder")
    .def(py::init<double, double>(), py::arg("radius"), py::arg("length"))
    .def_readonly("radius", &Cylinder::radius)
    .def_readonly("length", &Cylinder::length);
  py::class_<Mesh, Geometry>(m, "Mesh")
    .def(py::init<const std::string&, const std::string&>(),
         py::arg("uri"), py::arg("resolved_filename"))
    .def_readonly("scale", &Mesh::scale_)
    .def_readonly("uri", &Mesh::uri_)
    .def_readonly("resolved_filename", &Mesh::resolved_filename_);
  py::class_<MeshPoints, Geometry>(m, "MeshPoints")
    .def(py::init<const Eigen::Matrix3Xd&>(),
         py::arg("points"));
  py::class_<Capsule, Geometry>(m, "Capsule")
    .def(py::init<double, double>(), py::arg("radius"), py::arg("length"))
    .def_readonly("radius", &Capsule::radius)
    .def_readonly("length", &Capsule::length);

  py::class_<Element>(m, "Element")
    .def(py::init<const Geometry&, const Eigen::Isometry3d&>(),
         py::arg("geometry_in"),
         py::arg("T_element_to_local"))
    .def("hasGeometry", &Element::hasGeometry)
    .def("getGeometry", &Element::getGeometry, py_reference_internal)
    .def("getLocalTransform", [](const Element* self) {
      return self->getLocalTransform().matrix();
    });
  py::class_<VisualElement, Element>(m, "VisualElement")
    .def(py::init<const Geometry&, const Eigen::Isometry3d&,
                  const Eigen::Vector4d&>(),
         py::arg("geometry_in"),
         py::arg("T_element_to_local"),
         py::arg("material_in"))
    .def("setMaterial", &VisualElement::setMaterial, "Apply an RGBA material.")
    .def("getMaterial", &VisualElement::getMaterial, "Get an RGBA material.");
}

}  // namespace pydrake
}  // namespace drake
