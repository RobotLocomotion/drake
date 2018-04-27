#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"

#include "drake/automotive/maliput/api/junction.h"
#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/api/segment.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(api, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::maliput::api;

  m.doc() = "Bindings for the Maliput API.";

  // TODO(jadecastro) These bindings are work-in-progress. Expose additional
  // Maliput API features, as necessary (see #7918).

  py::class_<RoadGeometryId>(m, "RoadGeometryId")
      .def(py::init<std::string>())
      .def("string", &RoadGeometryId::string, py_reference_internal);

  py::class_<GeoPosition> (m, "GeoPosition")
      .def(py::init<double, double, double>(), py::arg("x"), py::arg("y"),
           py::arg("z"))
      .def("xyz", &GeoPosition::xyz, py_reference_internal);

  py::class_<LanePosition>(m, "LanePosition")
      .def(py::init<double, double, double>(), py::arg("s"), py::arg("r"),
           py::arg("h"))
      .def("srh", &LanePosition::srh, py_reference_internal);

  py::class_<RoadGeometry>(m, "RoadGeometry")
      .def("junction", &RoadGeometry::junction, py_reference_internal);

  py::class_<Junction>(m, "Junction")
      .def("segment", &Junction::segment, py_reference_internal);

  py::class_<Segment>(m, "Segment")
      .def("lane", &Segment::lane, py_reference_internal);

  py::class_<Lane>(m, "Lane")
      .def("ToLanePosition", &Lane::ToLanePosition)
      .def("ToGeoPosition", &Lane::ToGeoPosition);
}

}  // namespace pydrake
}  // namespace drake
