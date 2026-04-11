/* @file This contains the bounding box functions for pydrake.geometry. */

#include <set>
#include <utility>

#include "drake/bindings/generated_docstrings/geometry_proximity.h"
#include "drake/bindings/pydrake/geometry/geometry_py.h"
#include "drake/geometry/proximity/aabb.h"
#include "drake/geometry/proximity/calc_obb.h"
#include "drake/geometry/proximity/obb.h"
#include "drake/geometry/proximity/triangle_surface_mesh.h"
#include "drake/geometry/proximity/volume_mesh.h"

namespace drake {
namespace pydrake {

void DefineGeometryBoundingBox(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::geometry;
  constexpr auto& doc = pydrake_doc_geometry_proximity.drake.geometry;

  // Define Aabb class first.
  py::class_<Aabb> aabb_cls(m, "Aabb", doc.Aabb.doc);
  aabb_cls  // BR
      .def(py::init<const Vector3<double>&, const Vector3<double>&>(),
          py::arg("p_HoBo"), py::arg("half_width"), doc.Aabb.ctor.doc)
      .def("center", &Aabb::center, py_rvp::reference_internal,
          doc.Aabb.center.doc)
      .def("half_width", &Aabb::half_width, py_rvp::reference_internal,
          doc.Aabb.half_width.doc)
      .def("lower", &Aabb::lower, doc.Aabb.lower.doc)
      .def("upper", &Aabb::upper, doc.Aabb.upper.doc)
      .def("pose", &Aabb::pose, doc.Aabb.pose.doc)
      .def("CalcVolume", &Aabb::CalcVolume, doc.Aabb.CalcVolume.doc)
      .def("Equal", &Aabb::Equal, py::arg("other"), doc.Aabb.Equal.doc)
      .def(py::pickle(
          [](const Aabb& self) {
            return std::make_pair(self.center(), self.half_width());
          },
          [](std::pair<Vector3<double>, Vector3<double>> data) {
            return Aabb(data.first, data.second);
          }));
  DefCopyAndDeepCopy(&aabb_cls);

  // Define Obb class.
  py::class_<Obb> obb_cls(m, "Obb", doc.Obb.doc);
  obb_cls  // BR
      .def(py::init<const math::RigidTransformd&, const Vector3<double>&>(),
          py::arg("X_HB"), py::arg("half_width"), doc.Obb.ctor.doc)
      .def("center", &Obb::center, py_rvp::reference_internal,
          doc.Obb.center.doc)
      .def("half_width", &Obb::half_width, py_rvp::reference_internal,
          doc.Obb.half_width.doc)
      .def("pose", &Obb::pose, py_rvp::reference_internal, doc.Obb.pose.doc)
      .def("CalcVolume", &Obb::CalcVolume, doc.Obb.CalcVolume.doc)
      .def("Equal", &Obb::Equal, py::arg("other"), doc.Obb.Equal.doc)
      .def(py::pickle(
          [](const Obb& self) {
            return std::make_pair(self.pose(), self.half_width());
          },
          [](std::pair<math::RigidTransformd, Vector3<double>> data) {
            return Obb(data.first, data.second);
          }));
  DefCopyAndDeepCopy(&obb_cls);

  // Now add cross-referencing HasOverlap static methods.
  // Aabb static methods.
  aabb_cls.def_static("HasOverlap",
      py::overload_cast<const Aabb&, const Aabb&, const math::RigidTransformd&>(
          &Aabb::HasOverlap),
      py::arg("a_G"), py::arg("b_H"), py::arg("X_GH"),
      doc.Aabb.HasOverlap.doc_aabb_aabb);

  aabb_cls.def_static("HasOverlap",
      py::overload_cast<const Aabb&, const Obb&, const math::RigidTransformd&>(
          &Aabb::HasOverlap),
      py::arg("aabb_G"), py::arg("obb_H"), py::arg("X_GH"),
      doc.Aabb.HasOverlap.doc_aabb_obb);

  aabb_cls.def_static("HasOverlap",
      py::overload_cast<const Aabb&, const HalfSpace&,
          const math::RigidTransformd&>(&Aabb::HasOverlap),
      py::arg("bv_H"), py::arg("hs_C"), py::arg("X_CH"),
      doc.Aabb.HasOverlap.doc_aabb_halfspace);

  aabb_cls.def_static("HasOverlap",
      py::overload_cast<const Aabb&, const Plane<double>&,
          const math::RigidTransformd&>(&Aabb::HasOverlap),
      py::arg("bv_H"), py::arg("plane_P"), py::arg("X_PH"),
      doc.Aabb.HasOverlap.doc_aabb_plane);

  // Obb static methods.
  obb_cls.def_static("HasOverlap",
      py::overload_cast<const Obb&, const Obb&, const math::RigidTransformd&>(
          &Obb::HasOverlap),
      py::arg("a_G"), py::arg("b_H"), py::arg("X_GH"),
      doc.Obb.HasOverlap.doc_obb_obb);

  obb_cls.def_static("HasOverlap",
      py::overload_cast<const Obb&, const Aabb&, const math::RigidTransformd&>(
          &Obb::HasOverlap),
      py::arg("obb_G"), py::arg("aabb_H"), py::arg("X_GH"),
      doc.Obb.HasOverlap.doc_obb_aabb);

  obb_cls.def_static("HasOverlap",
      py::overload_cast<const Obb&, const HalfSpace&,
          const math::RigidTransformd&>(&Obb::HasOverlap),
      py::arg("bv_H"), py::arg("hs_C"), py::arg("X_CH"),
      doc.Obb.HasOverlap.doc_obb_halfspace);

  obb_cls.def_static("HasOverlap",
      py::overload_cast<const Obb&, const Plane<double>&,
          const math::RigidTransformd&>(&Obb::HasOverlap),
      py::arg("bv_H"), py::arg("plane_P"), py::arg("X_PH"),
      doc.Obb.HasOverlap.doc_obb_plane);

  // AabbMaker and ObbMaker utility functions
  // Instead of binding the classes directly (which have lifetime issues with
  // the std::set<int>& parameter), we provide simple functions that do the
  // complete workflow: construct, compute, and return the result.

  m.def(
      "ComputeAabbForTriangleMesh",
      [](const TriangleSurfaceMesh<double>& mesh_M,
          const std::set<int>& vertices) {
        AabbMaker<TriangleSurfaceMesh<double>> maker(mesh_M, vertices);
        return maker.Compute();
      },
      py::arg("mesh_M"), py::arg("vertices"),
      "Computes an axis-aligned bounding box for the specified vertices of a "
      "TriangleSurfaceMesh.");

  m.def(
      "ComputeAabbForVolumeMesh",
      [](const VolumeMesh<double>& mesh_M, const std::set<int>& vertices) {
        AabbMaker<VolumeMesh<double>> maker(mesh_M, vertices);
        return maker.Compute();
      },
      py::arg("mesh_M"), py::arg("vertices"),
      "Computes an axis-aligned bounding box for the specified vertices of a "
      "VolumeMesh.");

  m.def(
      "ComputeObbForTriangleMesh",
      [](const TriangleSurfaceMesh<double>& mesh_M,
          const std::set<int>& vertices) {
        ObbMaker<TriangleSurfaceMesh<double>> maker(mesh_M, vertices);
        return maker.Compute();
      },
      py::arg("mesh_M"), py::arg("vertices"),
      "Computes an oriented bounding box for the specified vertices of a "
      "TriangleSurfaceMesh.");

  m.def(
      "ComputeObbForVolumeMesh",
      [](const VolumeMesh<double>& mesh_M, const std::set<int>& vertices) {
        ObbMaker<VolumeMesh<double>> maker(mesh_M, vertices);
        return maker.Compute();
      },
      py::arg("mesh_M"), py::arg("vertices"),
      "Computes an oriented bounding box for the specified vertices of a "
      "VolumeMesh.");

  m.def("CalcObb", &CalcObb, py::arg("shape"), doc.CalcObb.doc);
}

}  // namespace pydrake
}  // namespace drake
