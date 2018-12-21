#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"

#include "drake/automotive/maliput/api/junction.h"
#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/api/segment.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/util/wrap_pybind.h"
#include "drake/math/roll_pitch_yaw.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(api, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::maliput::api;

  m.doc() = "Bindings for the Maliput API.";
  constexpr auto& doc = pydrake_doc.drake.maliput.api;

  // TODO(jadecastro) These bindings are work-in-progress. Expose additional
  // Maliput API features, as necessary (see #7918).

  // TODO(m-chaturvedi) Add doc when typedefs are parsed (#9599)
  py::class_<RoadGeometryId>(m, "RoadGeometryId")
      .def(py::init<std::string>(), doc.RoadGeometry.ctor.doc)
      .def("string", &RoadGeometryId::string, py_reference_internal);

  py::class_<GeoPosition>(m, "GeoPosition", doc.GeoPositionT.doc)
      .def(py::init<double, double, double>(), py::arg("x"), py::arg("y"),
          py::arg("z"), doc.GeoPositionT.ctor.doc_3args)
      .def("xyz", &GeoPosition::xyz, py_reference_internal,
          doc.GeoPositionT.xyz.doc);

  py::class_<LanePosition>(m, "LanePosition", doc.LanePositionT.doc)
      .def(py::init<double, double, double>(), py::arg("s"), py::arg("r"),
          py::arg("h"), doc.LanePositionT.ctor.doc_3args)
      .def("srh", &LanePosition::srh, py_reference_internal,
          doc.LanePositionT.srh.doc);

  py::class_<RoadPosition> road_position(
      m, "RoadPosition", doc.RoadPosition.doc);
  road_position  // BR
      .def(py::init<>(), doc.RoadPosition.ctor.doc_0args)
      .def(py::init<const Lane*, const LanePosition&>(), py::arg("lane"),
          py::arg("pos"),
          // Keep alive, reference: `self` keeps `Lane*` alive.
          py::keep_alive<1, 2>(), doc.RoadPosition.ctor.doc_2args)
      .def_readwrite("pos", &RoadPosition::pos, doc.RoadPosition.pos.doc);
  // TODO(m-chaturvedi) Add doc when typedefs are parsed (#9599)
  DefReadWriteKeepAlive(&road_position, "lane", &RoadPosition::lane);

  py::class_<Rotation>(m, "Rotation", doc.Rotation.doc)
      .def(
          "quat", &Rotation::quat, py_reference_internal, doc.Rotation.quat.doc)
      .def("rpy", &Rotation::rpy, doc.Rotation.rpy.doc);

  py::class_<RoadGeometry>(m, "RoadGeometry", doc.RoadGeometry.doc)
      .def("num_junctions", &RoadGeometry::num_junctions,
          doc.RoadGeometry.num_junctions.doc)
      .def("junction", &RoadGeometry::junction, py_reference_internal,
          doc.RoadGeometry.junction.doc);

  py::class_<Junction>(m, "Junction", doc.Junction.doc)
      .def("num_segments", &Junction::num_segments,
          doc.Junction.num_segments.doc)
      .def("segment", &Junction::segment, py_reference_internal,
          doc.Junction.segment.doc);

  py::class_<Segment>(m, "Segment", doc.Segment.doc)
      .def("num_lanes", &Segment::num_lanes, doc.Segment.num_lanes.doc)
      .def("lane", &Segment::lane, py_reference_internal, doc.Segment.lane.doc);

  // TODO(m-chaturvedi) Add Pybind11 documentation.
  py::class_<LaneId>(m, "LaneId")
      .def(py::init<std::string>(), doc.Lane.id.doc)
      .def("string", &LaneId::string, py_reference_internal);

  py::class_<Lane>(m, "Lane", doc.Lane.doc)
      .def("ToLanePosition", &Lane::ToLanePosition, doc.Lane.ToLanePosition.doc)
      .def("ToGeoPosition", &Lane::ToGeoPosition, doc.Lane.ToGeoPosition.doc)
      .def("GetOrientation", &Lane::GetOrientation, doc.Lane.GetOrientation.doc)
      .def("length", &Lane::length, doc.Lane.length.doc)
      .def("id", &Lane::id, doc.Lane.id.doc);
}

}  // namespace pydrake
}  // namespace drake
