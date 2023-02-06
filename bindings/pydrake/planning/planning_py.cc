#include "drake/bindings/pydrake/planning/planning_py.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(planning, m) {
  PYDRAKE_PREVENT_PYTHON3_MODULE_REIMPORT(m);

  m.doc() = R"""(
A collection of motion planning algorithms for finding configurations
and/or trajectories of dynamical systems.
)""";

  py::module::import("pydrake.geometry");
  py::module::import("pydrake.multibody.parsing");
  py::module::import("pydrake.multibody.plant");
  py::module::import("pydrake.systems.framework");

  // The order of these calls matters. Some modules rely on prior definitions.
  internal::DefinePlanningRobotDiagram(m);
  internal::DefinePlanningCollisionCheckerInterfaceTypes(m);
  internal::DefinePlanningCollisionChecker(m);
  internal::DefinePlanningTrajectoryOptimization(m);
}

}  // namespace pydrake
}  // namespace drake
