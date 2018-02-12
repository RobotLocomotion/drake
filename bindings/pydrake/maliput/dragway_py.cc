#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

#include "drake/automotive/maliput/dragway/lane.h"
#include "drake/automotive/maliput/dragway/junction.h"
#include "drake/automotive/maliput/dragway/road_geometry.h"
#include "drake/automotive/maliput/dragway/segment.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(dragway, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::maliput;

  m.doc() = "Bindings for the Dragway backend.";

  py::class_<dragway::RoadGeometry>(m, "RoadGeometry")
      .def(py::init<api::RoadGeometryId, int, double, double, double, double,
           double, double>())
      .def("junction", &dragway::RoadGeometry::junction, py_reference_internal,
           // Keep alive, reference: `return` keeps `self` alive.
           py::keep_alive<0, 1>());

  py::class_<dragway::Junction>(m, "Junction")
      .def("segment", &dragway::Junction::segment, py_reference_internal,
           // Keep alive, reference: `return` keeps `self` alive.
           py::keep_alive<0, 1>());

  py::class_<dragway::Segment>(m, "Segment")
      .def("lane", &dragway::Segment::lane, py_reference_internal,
           // Keep alive, reference: `return` keeps `self` alive.
           py::keep_alive<0, 1>());

  py::class_<dragway::Lane>(m, "Lane")
      .def("ToLanePosition", &dragway::Lane::ToLanePosition)
      .def("ToGeoPosition", &dragway::Lane::ToGeoPosition);
}

}  // namespace pydrake
}  // namespace drake
