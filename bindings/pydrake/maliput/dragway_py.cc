#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"

#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/dragway/road_geometry.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"

namespace drake {
namespace pydrake {

using std::make_unique;
using std::unique_ptr;

PYBIND11_MODULE(dragway, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::maliput;
  constexpr auto& doc = pydrake_doc.drake.maliput;

  m.doc() = "Bindings for the Dragway backend.";

  py::class_<dragway::RoadGeometry, api::RoadGeometry>(
      m, "RoadGeometry", doc.dragway.RoadGeometry.doc);

  m.def("create_dragway",
      [](api::RoadGeometryId road_id, int num_lanes, double length,
          double lane_width, double shoulder_width, double maximum_height,
          double linear_tolerance, double angular_tolerance) {
        return make_unique<dragway::RoadGeometry>(road_id, num_lanes, length,
            lane_width, shoulder_width, maximum_height, linear_tolerance,
            angular_tolerance);
      },
      py::arg("road_id"), py::arg("num_lanes"), py::arg("length"),
      py::arg("lane_width"), py::arg("shoulder_width"),
      py::arg("maximum_height"), py::arg("linear_tolerance"),
      py::arg("angular_tolerance"), doc.dragway.RoadGeometry.ctor.doc);
}

}  // namespace pydrake
}  // namespace drake
