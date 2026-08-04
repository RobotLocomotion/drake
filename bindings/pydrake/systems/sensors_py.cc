#include "drake/bindings/pydrake/systems/sensors_py.h"

namespace drake {
namespace pydrake {

PYDRAKE_MODULE(sensors, m) {
  PYDRAKE_PREVENT_PYTHON3_MODULE_REIMPORT(m);

  m.doc() = "Bindings for the sensors portion of the Systems framework.";

  py::module_::import_("pydrake.common.eigen_geometry");
  py::module_::import_("pydrake.common.schema");
  py::module_::import_("pydrake.geometry");
  py::module_::import_("pydrake.multibody.plant");
  py::module_::import_("pydrake.systems.framework");
  py::module_::import_("pydrake.systems.lcm");

  internal::DefineSensorsAccelerometer(m);
  internal::DefineSensorsGyroscope(m);
  internal::DefineSensorsImage(m);
  internal::DefineSensorsImageIo(m);
  internal::DefineSensorsRgbd(m);
  internal::DefineSensorsCameraConfig(m);
  internal::DefineSensorsLcm(m);
  internal::DefineSensorsRotaryEncoders(m);
}

}  // namespace pydrake
}  // namespace drake
