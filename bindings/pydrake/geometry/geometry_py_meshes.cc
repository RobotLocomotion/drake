/* @file This contains the various "atomic" mesh types and helper functions.
 They can be found in the pydrake.geometry module. */

#include <filesystem>
#include <vector>

#include "drake/bindings/generated_docstrings/geometry_proximity.h"
#include "drake/bindings/pydrake/common/default_scalars_pybind.h"
#include "drake/bindings/pydrake/common/type_pack.h"
#include "drake/bindings/pydrake/geometry/geometry_py.h"
#include "drake/geometry/proximity/obj_to_surface_mesh.h"
#include "drake/geometry/proximity/polygon_surface_mesh.h"
#include "drake/geometry/proximity/triangle_surface_mesh.h"
#include "drake/geometry/proximity/volume_mesh.h"
#include "drake/geometry/proximity/volume_to_surface_mesh.h"

namespace drake {
namespace pydrake {
namespace {

template <typename T>
void DoScalarDependentDefinitions(py::module m, T) {
  py::tuple param = GetPyParam<T>();
  constexpr auto& doc = pydrake_doc_geometry_proximity.drake.geometry;

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
    constexpr auto& cls_doc = doc.TriangleSurfaceMesh;
    auto cls = DefineTemplateClassWithDefault<Class>(
        m, "TriangleSurfaceMesh", param, cls_doc.doc);
    cls  // BR
        .def("element", &Class::element, py::arg("e"), cls_doc.element.doc)
        .def("vertex", &Class::vertex, py::arg("v"), cls_doc.vertex.doc)
        .def("num_vertices", &Class::num_vertices, cls_doc.num_vertices.doc)
        .def("num_elements", &Class::num_elements, cls_doc.num_elements.doc)
        .def(py::init<std::vector<SurfaceTriangle>, std::vector<Vector3<T>>>(),
            py::arg("triangles"), py::arg("vertices"), cls_doc.ctor.doc)
        .def("num_triangles", &Class::num_triangles, cls_doc.num_triangles.doc)
        .def("area", &Class::area, py::arg("t"), cls_doc.area.doc)
        .def("total_area", &Class::total_area, cls_doc.total_area.doc)
        .def("face_normal", &Class::face_normal, py::arg("t"),
            cls_doc.face_normal.doc)
        .def("triangles", &Class::triangles, cls_doc.triangles.doc)
        .def("vertices", &Class::vertices, cls_doc.vertices.doc)
        .def("centroid", &Class::centroid, cls_doc.centroid.doc)
        .def("element_centroid", &Class::element_centroid, py::arg("t"),
            cls_doc.element_centroid.doc)
        .def("CalcBoundingBox", &Class::CalcBoundingBox,
            cls_doc.CalcBoundingBox.doc)
        .def("Equal", &Class::Equal, py::arg("mesh"), cls_doc.Equal.doc)
        .def(
            "CalcCartesianFromBarycentric",
            [](const Class* self, int element_index, const Vector3<T>& b_Q) {
              return self->CalcCartesianFromBarycentric(element_index, b_Q);
            },
            py::arg("element_index"), py::arg("b_Q"),
            cls_doc.CalcCartesianFromBarycentric.doc)
        .def(
            "CalcBarycentric",
            [](const Class* self, const Vector3<T>& p_MQ, int t) {
              return self->CalcBarycentric(p_MQ, t);
            },
            py::arg("p_MQ"), py::arg("t"), cls_doc.CalcBarycentric.doc);
  }

  // VolumeMesh
  {
    using Class = VolumeMesh<T>;
    constexpr auto& cls_doc = doc.VolumeMesh;
    auto cls = DefineTemplateClassWithDefault<Class>(
        m, "VolumeMesh", param, doc.VolumeMesh.doc);
    cls  // BR
        .def(py::init<std::vector<VolumeElement>, std::vector<Vector3<T>>>(),
            py::arg("elements"), py::arg("vertices"), cls_doc.ctor.doc)
        .def("element", &Class::element, py::arg("e"), cls_doc.element.doc)
        .def("vertex", &Class::vertex, py::arg("v"), cls_doc.vertex.doc)
        .def("vertices", &Class::vertices, py_rvp::reference_internal,
            cls_doc.vertices.doc)
        .def("tetrahedra", &Class::tetrahedra, py_rvp::reference_internal,
            cls_doc.tetrahedra.doc)
        .def("num_elements", &Class::num_elements, cls_doc.num_elements.doc)
        .def("num_vertices", &Class::num_vertices, cls_doc.num_vertices.doc)
        .def("inward_normal", &Class::inward_normal, py::arg("e"), py::arg("f"),
            cls_doc.inward_normal.doc)
        .def("edge_vector", &Class::edge_vector, py::arg("e"), py::arg("a"),
            py::arg("b"), cls_doc.edge_vector.doc)
        .def("CalcTetrahedronVolume", &Class::CalcTetrahedronVolume,
            py::arg("e"), cls_doc.CalcTetrahedronVolume.doc)
        .def("CalcVolume", &Class::CalcVolume, cls_doc.CalcVolume.doc)
        .def(
            "CalcBarycentric",
            [](const Class* self, const Vector3<T>& p_MQ, int e) {
              return self->CalcBarycentric(p_MQ, e);
            },
            py::arg("p_MQ"), py::arg("e"), cls_doc.CalcBarycentric.doc)
        .def("Equal", &Class::Equal, py::arg("mesh"),
            py::arg("vertex_tolerance") = 0, cls_doc.Equal.doc);
  }

  m.def("ConvertVolumeToSurfaceMesh", &ConvertVolumeToSurfaceMesh<T>,
      py::arg("volume"), doc.ConvertVolumeToSurfaceMesh.doc);
}

void DoScalarIndependentDefinitions(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::geometry;
  constexpr auto& doc = pydrake_doc_geometry_proximity.drake.geometry;

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
        .def("num_vertices", &Class::num_vertices, cls_doc.num_vertices.doc)
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
        .def("vertex", &Class::vertex, py::arg("i"), cls_doc.vertex.doc)
        .def("num_vertices", &Class::num_vertices, cls_doc.num_vertices.doc);
    DefCopyAndDeepCopy(&cls);
  }
}

void DoMeshDependentDefinitions(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::geometry;
  constexpr auto& doc = pydrake_doc_geometry_proximity.drake.geometry;
  m.def(
      "ReadObjToTriangleSurfaceMesh",
      [](const std::filesystem::path& filename, double scale) {
        return geometry::ReadObjToTriangleSurfaceMesh(filename, scale);
      },
      py::arg("filename"), py::arg("scale") = 1.0,
      // N.B. We have not bound the optional "on_warning" argument.
      doc.ReadObjToTriangleSurfaceMesh.doc_3args_filename_scale_on_warning);
  m.def(
      "ReadObjToTriangleSurfaceMesh",
      [](const std::filesystem::path& filename, const Eigen::Vector3d& scale3) {
        return geometry::ReadObjToTriangleSurfaceMesh(filename, scale3);
      },
      py::arg("filename"), py::arg("scale3"),
      // N.B. We have not bound the optional "on_warning" argument.
      doc.ReadObjToTriangleSurfaceMesh.doc_3args_filename_scale3_on_warning);
  // Note: we're not binding the MeshSource variants to avoid having to resolve
  // the binding dependency chain on MeshSource.
}

}  // namespace

void DefineGeometryMeshes(py::module m) {
  DoScalarIndependentDefinitions(m);
  type_visit([m](auto dummy) { DoScalarDependentDefinitions(m, dummy); },
      NonSymbolicScalarPack{});
  DoMeshDependentDefinitions(m);
}
}  // namespace pydrake
}  // namespace drake
