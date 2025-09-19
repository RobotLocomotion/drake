/* @file This contains the various public components of the hydroelastic contact
 calculations -- specifically, the mesh field classes and property helper
 functions. They can be found in the pydrake.geometry module. */

#include "drake/bindings/generated_docstrings/geometry.h"
#include "drake/bindings/generated_docstrings/geometry_proximity.h"
#include "drake/bindings/pydrake/common/default_scalars_pybind.h"
#include "drake/bindings/pydrake/common/type_pack.h"
#include "drake/bindings/pydrake/geometry/geometry_py.h"
#include "drake/geometry/proximity/make_convex_hull_mesh.h"
#include "drake/geometry/proximity/polygon_surface_mesh_field.h"
#include "drake/geometry/proximity/triangle_surface_mesh_field.h"
#include "drake/geometry/proximity_properties.h"

namespace drake {
namespace pydrake {
namespace {

template <typename T>
void DoScalarDependentDefinitions(py::module m, T) {
  py::tuple param = GetPyParam<T>();
  constexpr auto& doc = pydrake_doc_geometry_proximity.drake.geometry;

  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::geometry;

  /* A note on the bindings of ***SurfaceMeshFieldLinear.

   The current binding's purpose is to allow the user to view the fields
   associated with a ContactSurface. As such:

     - We don't bind the constructors.
     - We are only binding *scalar* fields (where the `FieldType` is the same
       scalar type as the vertex position scalar type). This is sufficient for
       what ContactSurface uses.
     - We only bind the Evaluate*** methods.
       - This leaves out
         - Transform() (already marked "Advanced" in documentation)
         - CloneAndSetMesh()
         - mesh() and values() accessors
         - Equal()
     - We steal the documentation from MeshFieldLinear.

   If we need to support vector fields of arbitrary higher dimension in the
   future, we can revisit these bindings.

   Because we can't construct them, we test them in conjunction with testing
   ContactSurface in geometry_scene_graph_test.py.

   Differences between the SurfaceMeshFields for Triangle and Polygon:
     Polygon:
       - Always has gradients, so EvalauteGradient() is bound.
       - Has no barycentric coordinates, so Evaluate() is not bound (it would
         only throw).
     Triangle:
       - Has no gradients computed in the creation of a ContactSurface, so
         EvaluateGradient() is not bound (it would only throw).
       - Has barycentric coordinates, so Evaluate() is bound.
   */

  // PolygonSurfaceMeshFieldLinear
  // Currently we do not bind the constructor because users do not need to
  // construct it directly yet. We can get it from ContactSurface.
  {
    using Class = PolygonSurfaceMeshFieldLinear<T, T>;
    constexpr auto& cls_doc = doc.MeshFieldLinear;
    auto cls = DefineTemplateClassWithDefault<Class>(
        m, "PolygonSurfaceMeshFieldLinear", GetPyParam<T, T>(), cls_doc.doc);
    cls  // BR
        .def("EvaluateAtVertex", &Class::EvaluateAtVertex, py::arg("v"),
            cls_doc.EvaluateAtVertex.doc)
        .def("EvaluateGradient", &Class::EvaluateGradient, py::arg("e"),
            cls_doc.EvaluateGradient.doc)
        .def(
            "EvaluateCartesian",
            [](const Class* self, int e, const Vector3<T>& p_MQ) {
              return self->EvaluateCartesian(e, p_MQ);
            },
            py::arg("e"), py::arg("p_MQ"), cls_doc.EvaluateCartesian.doc);
  }

  // TriangleSurfaceMeshFieldLinear
  // See also "A note on the bindings of ***SurfaceMeshFieldLinear" above.
  // Currently we do not bind the constructor because users do not need to
  // construct it directly yet. We can get it from ContactSurface.
  {
    using Class = TriangleSurfaceMeshFieldLinear<T, T>;
    constexpr auto& cls_doc = doc.MeshFieldLinear;
    auto cls = DefineTemplateClassWithDefault<Class>(
        m, "TriangleSurfaceMeshFieldLinear", GetPyParam<T, T>(), cls_doc.doc);
    using Barycentric =
        typename TriangleSurfaceMesh<T>::template Barycentric<T>;
    cls  // BR
        .def("EvaluateAtVertex", &Class::EvaluateAtVertex, py::arg("v"),
            cls_doc.EvaluateAtVertex.doc)
        .def(
            "Evaluate",
            [](const Class* self, int e, const Barycentric& b) {
              return self->Evaluate(e, b);
            },
            py::arg("e"), py::arg("b"), cls_doc.Evaluate.doc)
        .def(
            "EvaluateCartesian",
            [](const Class* self, int e, const Vector3<T>& p_MQ) {
              return self->EvaluateCartesian(e, p_MQ);
            },
            py::arg("e"), py::arg("p_MQ"), cls_doc.EvaluateCartesian.doc);
  }
}

void DoScalarIndependentDefinitions(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::geometry;
  constexpr auto& doc = pydrake_doc_geometry.drake.geometry;

  // MakeConvexHull
  {
    constexpr char internal_doc[] = "(internal use only)";
    m.def("_MakeConvexHull", &drake::geometry::internal::MakeConvexHull,
        py::arg("shape"), internal_doc);
  }

  m.def("AddRigidHydroelasticProperties",
      py::overload_cast<double, ProximityProperties*>(
          &AddRigidHydroelasticProperties),
      py::arg("resolution_hint"), py::arg("properties"),
      doc.AddRigidHydroelasticProperties.doc_2args);

  m.def("AddRigidHydroelasticProperties",
      py::overload_cast<ProximityProperties*>(&AddRigidHydroelasticProperties),
      py::arg("properties"), doc.AddRigidHydroelasticProperties.doc_1args);

  m.def("AddCompliantHydroelasticProperties",
      &AddCompliantHydroelasticProperties, py::arg("resolution_hint"),
      py::arg("hydroelastic_modulus"), py::arg("properties"),
      doc.AddCompliantHydroelasticProperties.doc);

  m.def("AddCompliantHydroelasticPropertiesForHalfSpace",
      &AddCompliantHydroelasticPropertiesForHalfSpace,
      py::arg("slab_thickness"), py::arg("hydroelastic_modulus"),
      py::arg("properties"),
      doc.AddCompliantHydroelasticPropertiesForHalfSpace.doc);
}

}  // namespace

void DefineGeometryHydro(py::module m) {
  DoScalarIndependentDefinitions(m);
  type_visit([m](auto dummy) { DoScalarDependentDefinitions(m, dummy); },
      NonSymbolicScalarPack{});
}
}  // namespace pydrake
}  // namespace drake
