/* @file This contains the various public components of the hydroelastic contact
 calculations -- specifically, the mesh types and basic I/O operations on those
 types. The can be found in the pydrake.geometry module. */

#include "drake/bindings/pydrake/common/default_scalars_pybind.h"
#include "drake/bindings/pydrake/common/type_pack.h"
#include "drake/bindings/pydrake/common/type_safe_index_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/geometry/geometry_roles.h"
#include "drake/geometry/proximity/obj_to_surface_mesh.h"
#include "drake/geometry/proximity/surface_mesh.h"
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

  // SurfaceMesh
  {
    using Class = SurfaceMesh<T>;
    auto cls = DefineTemplateClassWithDefault<Class>(
        m, "SurfaceMesh", param, doc.SurfaceMesh.doc);
    cls  // BR
        .def(py::init<std::vector<SurfaceFace>, std::vector<Vector3<T>>>(),
            py::arg("faces"), py::arg("vertices"), doc.SurfaceMesh.ctor.doc)
        .def("faces", &Class::faces, doc.SurfaceMesh.faces.doc)
        .def("vertices", &Class::vertices, doc.SurfaceMesh.vertices.doc)
        .def("centroid", &Class::centroid, doc.SurfaceMesh.centroid.doc);
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

  // SurfaceFace
  {
    using Class = SurfaceFace;
    constexpr auto& cls_doc = doc.SurfaceFace;
    py::class_<Class> cls(m, "SurfaceFace", cls_doc.doc);
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
      "ReadObjToSurfaceMesh",
      [](const std::string& filename, double scale) {
        return geometry::ReadObjToSurfaceMesh(filename, scale);
      },
      py::arg("filename"), py::arg("scale") = 1.0,
      // N.B. We have not bound the optional "on_warning" argument.
      doc.ReadObjToSurfaceMesh.doc_3args_filename_scale_on_warning);

  m.def("AddRigidHydroelasticProperties",
      py::overload_cast<double, ProximityProperties*>(
          &AddRigidHydroelasticProperties),
      py::arg("resolution_hint"), py::arg("properties"),
      doc.AddRigidHydroelasticProperties.doc_2args);

  m.def("AddRigidHydroelasticProperties",
      py::overload_cast<ProximityProperties*>(&AddRigidHydroelasticProperties),
      py::arg("properties"), doc.AddRigidHydroelasticProperties.doc_1args);

  m.def("AddSoftHydroelasticProperties",
      py::overload_cast<double, ProximityProperties*>(
          &AddSoftHydroelasticProperties),
      py::arg("resolution_hint"), py::arg("properties"),
      doc.AddSoftHydroelasticProperties.doc_2args);

  m.def("AddSoftHydroelasticProperties",
      py::overload_cast<ProximityProperties*>(&AddSoftHydroelasticProperties),
      py::arg("properties"), doc.AddSoftHydroelasticProperties.doc_1args);

  m.def("AddSoftHydroelasticPropertiesForHalfSpace",
      &AddSoftHydroelasticPropertiesForHalfSpace, py::arg("slab_thickness"),
      py::arg("properties"), doc.AddSoftHydroelasticPropertiesForHalfSpace.doc);
}

}  // namespace

void DefineGeometryHydro(py::module m) {
  DoScalarIndependentDefinitions(m);
  type_visit([m](auto dummy) { DoScalarDependentDefinitions(m, dummy); },
      NonSymbolicScalarPack{});
}
}  // namespace pydrake
}  // namespace drake
