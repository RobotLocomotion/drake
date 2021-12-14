/* @file This contains the various public components of the hydroelastic contact
 calculations -- specifically, the mesh types and basic I/O operations on those
 types. The can be found in the pydrake.geometry module. */

#include "drake/bindings/pydrake/common/default_scalars_pybind.h"
#include "drake/bindings/pydrake/common/type_pack.h"
#include "drake/bindings/pydrake/common/type_safe_index_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/geometry/geometry_roles.h"
#include "drake/geometry/proximity/obj_to_surface_mesh.h"
#include "drake/geometry/proximity/polygon_surface_mesh.h"
#include "drake/geometry/proximity/triangle_surface_mesh.h"
#include "drake/geometry/proximity/volume_mesh.h"
#include "drake/geometry/proximity/volume_to_surface_mesh.h"
#include "drake/geometry/proximity_properties.h"

namespace drake {
namespace pydrake {
namespace {

template <typename T>
void DoScalarDependentDefinitions(py::module m, T) {
  py::tuple param = GetPyParam<T>();
  constexpr auto& doc = pydrake_doc.drake.geometry;

  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::geometry;

  // PolygonSurfaceMesh
  {
    using Class = PolygonSurfaceMesh<T>;
    constexpr auto& cls_doc = doc.PolygonSurfaceMesh;
    auto cls = DefineTemplateClassWithDefault<Class>(
        m, "PolygonSurfaceMesh", param, cls_doc.doc);
    cls  // BR
        .def(
            "element",
            [](Class* self, int e) {
              // The SurfacePolygon class is non-copyable, so we need to do a
              // little dance here in order to bind it into pydrake.
              return self->element(e).copy_to_unique();
            },
            py::arg("e"),
            // The return value is a view into the PolygonSurfaceMesh storage.
            py::keep_alive<0, 1>(), cls_doc.element.doc)
        .def("vertex", &Class::vertex, py::arg("v"), cls_doc.vertex.doc)
        .def("num_vertices", &Class::num_vertices, cls_doc.num_vertices.doc)
        .def("num_elements", &Class::num_elements, cls_doc.num_elements.doc)
        .def(py::init<>(), cls_doc.ctor.doc_0args)
        .def(py::init<std::vector<int>, std::vector<Vector3<T>>>(),
            py::arg("face_data"), py::arg("vertices"), cls_doc.ctor.doc_2args)
        .def("TransformVertices", &Class::TransformVertices, py::arg("X_NM"),
            cls_doc.TransformVertices.doc)
        .def("ReverseFaceWinding", &Class::ReverseFaceWinding,
            cls_doc.ReverseFaceWinding.doc)
        .def("num_faces", &Class::num_faces, cls_doc.num_faces.doc)
        .def("area", &Class::area, py::arg("f"), cls_doc.area.doc)
        .def("total_area", &Class::total_area, cls_doc.total_area.doc)
        .def("face_normal", &Class::face_normal, py::arg("f"),
            cls_doc.face_normal.doc)
        .def("element_centroid", &Class::element_centroid, py::arg("e"),
            cls_doc.element_centroid.doc)
        .def("centroid", &Class::centroid, cls_doc.centroid.doc)
        .def("CalcBoundingBox", &Class::CalcBoundingBox,
            cls_doc.CalcBoundingBox.doc)
        .def("Equal", &Class::Equal, py::arg("mesh"), cls_doc.Equal.doc)
        .def("face_data", &Class::face_data, cls_doc.face_data.doc);
    DefCopyAndDeepCopy(&cls);
  }

  // TriangleSurfaceMesh
  {
    using Class = TriangleSurfaceMesh<T>;
    auto cls = DefineTemplateClassWithDefault<Class>(
        m, "TriangleSurfaceMesh", param, doc.TriangleSurfaceMesh.doc);
    cls  // BR
        .def(py::init<std::vector<SurfaceTriangle>, std::vector<Vector3<T>>>(),
            py::arg("triangles"), py::arg("vertices"),
            doc.TriangleSurfaceMesh.ctor.doc)
        .def("triangles", &Class::triangles,
            doc.TriangleSurfaceMesh.triangles.doc)
        .def("vertices", &Class::vertices, doc.TriangleSurfaceMesh.vertices.doc)
        .def("centroid", &Class::centroid, doc.TriangleSurfaceMesh.centroid.doc)
        .def("element_centroid", &Class::element_centroid, py::arg("t"),
            doc.TriangleSurfaceMesh.element_centroid.doc);
  }

  // VolumeMesh
  {
    using Class = VolumeMesh<T>;
    auto cls = DefineTemplateClassWithDefault<Class>(
        m, "VolumeMesh", param, doc.VolumeMesh.doc);
    cls  // BR
        .def(py::init<std::vector<VolumeElement>, std::vector<Vector3<T>>>(),
            py::arg("elements"), py::arg("vertices"), doc.VolumeMesh.ctor.doc)
        .def("vertices", &Class::vertices, py_rvp::reference_internal,
            doc.VolumeMesh.vertices.doc)
        .def("tetrahedra", &Class::tetrahedra, py_rvp::reference_internal,
            doc.VolumeMesh.tetrahedra.doc)
        .def("CalcTetrahedronVolume", &Class::CalcTetrahedronVolume,
            py::arg("e"), doc.VolumeMesh.CalcTetrahedronVolume.doc)
        .def("CalcVolume", &Class::CalcVolume, doc.VolumeMesh.CalcVolume.doc);
  }

  m.def("ConvertVolumeToSurfaceMesh", &ConvertVolumeToSurfaceMesh<T>,
      py::arg("volume"), doc.ConvertVolumeToSurfaceMesh.doc);
}

void DoScalarIndependentDefinitions(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::geometry;
  constexpr auto& doc = pydrake_doc.drake.geometry;

  // SurfacePolygon
  {
    using Class = SurfacePolygon;
    constexpr auto& cls_doc = doc.SurfacePolygon;
    py::class_<Class> cls(m, "SurfacePolygon", cls_doc.doc);
    cls  // BR
        .def("num_vertices", &Class::num_vertices, cls_doc.num_vertices.doc)
        .def("vertex", &Class::vertex, py::arg("i"), cls_doc.vertex.doc);
  }

  // SurfaceTriangle
  {
    using Class = SurfaceTriangle;
    constexpr auto& cls_doc = doc.SurfaceTriangle;
    py::class_<Class> cls(m, "SurfaceTriangle", cls_doc.doc);
    cls  // BR
        .def(py::init<int, int, int>(), py::arg("v0"), py::arg("v1"),
            py::arg("v2"), cls_doc.ctor.doc_3args)
        // TODO(SeanCurtis-TRI): Bind constructor that takes array of ints.
        .def("vertex", &Class::vertex, py::arg("i"), cls_doc.vertex.doc);
    DefCopyAndDeepCopy(&cls);
  }

  // VolumeElement
  {
    using Class = VolumeElement;
    constexpr auto& cls_doc = doc.VolumeElement;
    py::class_<Class> cls(m, "VolumeElement", cls_doc.doc);
    cls  // BR
        .def(py::init<int, int, int, int>(), py::arg("v0"), py::arg("v1"),
            py::arg("v2"), py::arg("v3"), cls_doc.ctor.doc_4args)
        // TODO(SeanCurtis-TRI): Bind constructor that takes array of ints.
        .def("vertex", &Class::vertex, py::arg("i"), cls_doc.vertex.doc);
    DefCopyAndDeepCopy(&cls);
  }

  m.def(
      "ReadObjToTriangleSurfaceMesh",
      [](const std::string& filename, double scale) {
        return geometry::ReadObjToTriangleSurfaceMesh(filename, scale);
      },
      py::arg("filename"), py::arg("scale") = 1.0,
      // N.B. We have not bound the optional "on_warning" argument.
      doc.ReadObjToTriangleSurfaceMesh.doc_3args_filename_scale_on_warning);

  m.def("AddRigidHydroelasticProperties",
      py::overload_cast<double, ProximityProperties*>(
          &AddRigidHydroelasticProperties),
      py::arg("resolution_hint"), py::arg("properties"),
      doc.AddRigidHydroelasticProperties.doc_2args);

  m.def("AddRigidHydroelasticProperties",
      py::overload_cast<ProximityProperties*>(&AddRigidHydroelasticProperties),
      py::arg("properties"), doc.AddRigidHydroelasticProperties.doc_1args);

  m.def("AddSoftHydroelasticProperties", &AddSoftHydroelasticProperties,
      py::arg("resolution_hint"), py::arg("hydroelastic_modulus"),
      py::arg("properties"), doc.AddSoftHydroelasticProperties.doc);

  m.def("AddSoftHydroelasticPropertiesForHalfSpace",
      &AddSoftHydroelasticPropertiesForHalfSpace, py::arg("slab_thickness"),
      py::arg("hydroelastic_modulus"), py::arg("properties"),
      doc.AddSoftHydroelasticPropertiesForHalfSpace.doc);
}

}  // namespace

void DefineGeometryHydro(py::module m) {
  DoScalarIndependentDefinitions(m);
  type_visit([m](auto dummy) { DoScalarDependentDefinitions(m, dummy); },
      NonSymbolicScalarPack{});
}
}  // namespace pydrake
}  // namespace drake
