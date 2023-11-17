#include "drake/bindings/pydrake/systems/sensors_py.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(sensors, m) {
  PYDRAKE_PREVENT_PYTHON3_MODULE_REIMPORT(m);

  m.doc() = "Bindings for the sensors portion of the Systems framework.";

  py::module::import("pydrake.common.eigen_geometry");
  py::module::import("pydrake.common.schema");
  py::module::import("pydrake.geometry");
  py::module::import("pydrake.systems.framework");

  internal::DefineSensorsImage(m);
  internal::DefineSensorsImageIo(m);
  internal::DefineSensorsRgbd(m);
  internal::DefineSensorsCameraConfig(m);
  internal::DefineSensorsLcm(m);
}

}  // namespace pydrake
}  // namespace drake
