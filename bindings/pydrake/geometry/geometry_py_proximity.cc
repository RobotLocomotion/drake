/* @file This contains the proximity types for pydrake.geometry. */

#include <set>
#include <utility>

#include "drake/bindings/generated_docstrings/geometry_proximity.h"
#include "drake/bindings/pydrake/common/default_scalars_pybind.h"
#include "drake/bindings/pydrake/common/type_pack.h"
#include "drake/bindings/pydrake/geometry/geometry_py.h"
#include "drake/geometry/proximity/plane.h"

namespace drake {
namespace pydrake {

// NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
using namespace drake::geometry;
constexpr auto& doc = pydrake_doc_geometry_proximity.drake.geometry;

template <typename T>
void DefinePlane(py::module m, T) {
  py::tuple param = GetPyParam<T>();
  {
    using Class = Plane<T>;
    constexpr auto& cls_doc = doc.Plane;
    auto cls =
        DefineTemplateClassWithDefault<Class>(m, "Plane", param, cls_doc.doc);
    cls  // BR
        .def(py::init<const Vector3<T>&, const Vector3<T>&, bool>(),
            py::arg("normal"), py::arg("point_on_plane"),
            py::arg("already_normalized") = false, cls_doc.ctor.doc)
        .def(
            "CalcHeight",
            [](const Class* self, const Vector3<T>& point) {
              return self->CalcHeight(point);
            },
            py::arg("point"), cls_doc.CalcHeight.doc)
        .def("normal", &Class::normal, py_rvp::reference_internal,
            cls_doc.normal.doc)
        .def("point_on_plane", &Class::point_on_plane,
            cls_doc.point_on_plane.doc)
        .def("BoxOverlaps", &Class::BoxOverlaps, py::arg("half_width"),
            py::arg("box_center_in_plane"), py::arg("box_orientation_in_plane"),
            cls_doc.BoxOverlaps.doc)
        .def(py::pickle(
            [](const Plane<T>& self) {
              return std::make_pair(self.normal(), self.point_on_plane());
            },
            [](std::pair<Vector3<T>, Vector3<T>> data) {
              return Plane<T>(data.first, data.second);
            }));
    DefCopyAndDeepCopy(&cls);
  }
}

void DefineGeometryProximity(py::module m) {
  type_visit(
      [m](auto dummy) {
        // This list must remain in topological dependency order.
        DefinePlane(m, dummy);
      },
      NonSymbolicScalarPack{});
}

}  // namespace pydrake
}  // namespace drake
