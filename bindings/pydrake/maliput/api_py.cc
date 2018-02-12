#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(api, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::maliput::api;

  m.doc() = "Bindings for the Maliput API.";

  py::class_<RoadGeometryId>(m, "RoadGeometryId")
      .def(py::init<std::string>());

  py::class_<GeoPosition> (m, "GeoPosition")
      .def(py::init<double, double, double>(), py::arg("x"), py::arg("y"),
           py::arg("z"))
      .def("xyz", &GeoPosition::xyz);

  py::class_<LanePosition>(m, "LanePosition")
      .def(py::init<double, double, double>(), py::arg("s"), py::arg("r"),
           py::arg("h"))
      .def("srh", &LanePosition::srh);
}

}  // namespace pydrake
}  // namespace drake
