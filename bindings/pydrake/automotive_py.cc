#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

#include "drake/automotive/calc_ongoing_road_position.h"
#include "drake/automotive/gen/driving_command.h"
#include "drake/automotive/idm_controller.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/pure_pursuit_controller.h"
#include "drake/automotive/simple_car.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/systems/systems_pybind.h"
#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(automotive, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::systems;
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::automotive;

  m.doc() = "Bindings for Automotive systems";

  py::module::import("pydrake.systems.framework");
  py::module::import("pydrake.systems.rendering");

  using T = double;

  py::enum_<RoadPositionStrategy>(m, "RoadPositionStrategy")
      .value("kCache", RoadPositionStrategy::kCache)
      .value("kExhaustiveSearch", RoadPositionStrategy::kExhaustiveSearch);

  py::enum_<ScanStrategy>(m, "ScanStrategy")
      .value("kPath", ScanStrategy::kPath)
      .value("kBranches", ScanStrategy::kBranches);

  py::class_<LaneDirection>(m, "LaneDirection")
      .def(py::init<const maliput::api::Lane*, bool>(), py::arg("lane"),
           py::arg("with_s"))
      .def_readwrite("lane", &LaneDirection::lane)
      .def_readwrite("with_s", &LaneDirection::with_s);
  pysystems::AddValueInstantiation<LaneDirection>(m);

  // TODO(eric.cousineau) Bind this named vector automatically (see #8096).
  py::class_<DrivingCommand<T>, BasicVector<T>>(m, "DrivingCommand")
      .def(py::init<>())
      .def("steering_angle", &DrivingCommand<T>::steering_angle)
      .def("acceleration", &DrivingCommand<T>::acceleration)
      .def("set_steering_angle", &DrivingCommand<T>::set_steering_angle)
      .def("set_acceleration", &DrivingCommand<T>::set_acceleration);

  py::class_<IdmController<T>, LeafSystem<T>>(m, "IdmController")
      .def(py::init<const maliput::api::RoadGeometry&,
           ScanStrategy, RoadPositionStrategy, double>(), py::arg("road"),
           py::arg("path_or_branches"), py::arg("road_position_strategy"),
           py::arg("period_sec"))
      .def("ego_pose_input", &IdmController<T>::ego_pose_input,
           py_reference_internal)
      .def("ego_velocity_input", &IdmController<T>::ego_velocity_input,
           py_reference_internal)
      .def("traffic_input", &IdmController<T>::traffic_input,
           py_reference_internal)
      .def("acceleration_output", &IdmController<T>::acceleration_output,
           py_reference_internal);

  py::class_<PurePursuitController<T>, LeafSystem<T>>(
      m, "PurePursuitController")
      .def(py::init<>())
      .def("ego_pose_input", &PurePursuitController<T>::ego_pose_input,
           py_reference_internal)
      .def("lane_input", &PurePursuitController<T>::lane_input,
           py_reference_internal)
      .def("steering_command_output",
           &PurePursuitController<T>::steering_command_output,
           py_reference_internal);

  // TODO(eric.cousineau) Bind this named vector automatically (see #8096).
  py::class_<SimpleCarState<T>, BasicVector<T>>(m, "SimpleCarState")
      .def(py::init<>())
      .def("x", &SimpleCarState<T>::x)
      .def("y", &SimpleCarState<T>::y)
      .def("heading", &SimpleCarState<T>::heading)
      .def("velocity", &SimpleCarState<T>::velocity)
      .def("set_x", &SimpleCarState<T>::set_x)
      .def("set_y", &SimpleCarState<T>::set_y)
      .def("set_heading", &SimpleCarState<T>::set_heading)
      .def("set_velocity", &SimpleCarState<T>::set_velocity);

  py::class_<SimpleCar<T>, LeafSystem<T>>(m, "SimpleCar")
      .def(py::init<>())
      .def("state_output", &SimpleCar<T>::state_output, py_reference_internal)
      .def("pose_output", &SimpleCar<T>::pose_output, py_reference_internal)
      .def("velocity_output", &SimpleCar<T>::velocity_output,
           py_reference_internal);

  // TODO(jadecastro) Bind more systems as appropriate.
}

}  // namespace pydrake
}  // namespace drake
