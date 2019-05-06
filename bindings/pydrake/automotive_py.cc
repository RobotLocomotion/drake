#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

#include "drake/automotive/calc_ongoing_road_position.h"
#include "drake/automotive/gen/driving_command.h"
#include "drake/automotive/idm_controller.h"
#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/pose_selector.h"
#include "drake/automotive/pure_pursuit_controller.h"
#include "drake/automotive/road_odometry.h"
#include "drake/automotive/simple_car.h"
#include "drake/bindings/pydrake/common/value_pybind.h"
#include "drake/bindings/pydrake/common/wrap_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(automotive, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::systems;
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::automotive;

  m.doc() = "Bindings for Automotive systems";
  constexpr auto& doc = pydrake_doc.drake.automotive;

  py::module::import("pydrake.systems.framework");
  py::module::import("pydrake.systems.rendering");

  using T = double;

  py::enum_<AheadOrBehind>(m, "AheadOrBehind", doc.AheadOrBehind.doc)
      .value("kAhead", AheadOrBehind::kAhead, doc.AheadOrBehind.kAhead.doc)
      .value("kBehind", AheadOrBehind::kBehind, doc.AheadOrBehind.kBehind.doc);

  py::enum_<RoadPositionStrategy>(
      m, "RoadPositionStrategy", doc.RoadPositionStrategy.doc)
      .value("kCache", RoadPositionStrategy::kCache,
          doc.RoadPositionStrategy.kCache.doc)
      .value("kExhaustiveSearch", RoadPositionStrategy::kExhaustiveSearch,
          doc.RoadPositionStrategy.kExhaustiveSearch.doc);

  py::enum_<ScanStrategy>(m, "ScanStrategy", doc.ScanStrategy.doc)
      .value("kPath", ScanStrategy::kPath, doc.ScanStrategy.kPath.doc)
      .value(
          "kBranches", ScanStrategy::kBranches, doc.ScanStrategy.kBranches.doc);

  py::class_<ClosestPose<T>>(m, "ClosestPose", doc.ClosestPose.doc)
      .def(py::init<>(), doc.ClosestPose.ctor.doc_0args)
      .def(py::init<const RoadOdometry<T>&, const T&>(), py::arg("odom"),
          py::arg("dist"),
          // Keep alive, transitive: `self` keeps `RoadOdometry` pointer
          // members alive.
          py::keep_alive<1, 2>(), doc.ClosestPose.ctor.doc_2args)
      .def_readwrite("odometry", &ClosestPose<T>::odometry,
          py_reference_internal, doc.ClosestPose.odometry.doc)
      .def_readwrite(
          "distance", &ClosestPose<T>::distance, doc.ClosestPose.distance.doc);

  py::class_<RoadOdometry<T>> road_odometry(
      m, "RoadOdometry", doc.RoadOdometry.doc);
  road_odometry  // BR
      .def(py::init<>(), doc.RoadOdometry.ctor.doc_0args)
      .def(py::init<const maliput::api::RoadPosition&,
               const systems::rendering::FrameVelocity<T>&>(),
          py::arg("road_position"), py::arg("frame_velocity"),
          // Keep alive, transitive: `self` keeps `RoadPosition` pointer
          // members alive.
          py::keep_alive<1, 2>(), doc.RoadOdometry.ctor.doc_2args)
      .def(py::init<const maliput::api::Lane*,
               const maliput::api::LanePositionT<T>&,
               const systems::rendering::FrameVelocity<T>&>(),
          py::arg("lane"), py::arg("lane_position"), py::arg("frame_velocity"),
          // Keep alive, reference: `self` keeps `Lane*` alive.
          py::keep_alive<1, 2>(), doc.RoadOdometry.ctor.doc_3args)
      .def_readwrite("pos", &RoadOdometry<T>::pos, doc.RoadOdometry.pos.doc)
      .def_readwrite("vel", &RoadOdometry<T>::vel, doc.RoadOdometry.vel.doc);
  // TODO(m-chaturvedi) Add Pybind11 documentation.
  DefReadWriteKeepAlive(&road_odometry, "lane", &RoadOdometry<T>::lane);

  py::class_<LaneDirection>(m, "LaneDirection", doc.LaneDirection.doc)
      .def(py::init<const maliput::api::Lane*, bool>(), py::arg("lane"),
          py::arg("with_s"), doc.LaneDirection.ctor.doc_2args)
      .def_readwrite("lane", &LaneDirection::lane, py_reference_internal,
          doc.LaneDirection.lane.doc)
      .def_readwrite(
          "with_s", &LaneDirection::with_s, doc.LaneDirection.with_s.doc);
  AddValueInstantiation<LaneDirection>(m);

  // TODO(eric.cousineau) Bind this named vector automatically (see #8096).
  py::class_<DrivingCommand<T>, BasicVector<T>>(
      m, "DrivingCommand", doc.DrivingCommand.doc)
      .def(py::init<>(), doc.DrivingCommand.ctor.doc)
      .def("steering_angle", &DrivingCommand<T>::steering_angle,
          doc.DrivingCommand.steering_angle.doc)
      .def("acceleration", &DrivingCommand<T>::acceleration,
          doc.DrivingCommand.acceleration.doc)
      .def("set_steering_angle", &DrivingCommand<T>::set_steering_angle,
          doc.DrivingCommand.set_steering_angle.doc)
      .def("set_acceleration", &DrivingCommand<T>::set_acceleration,
          doc.DrivingCommand.set_acceleration.doc);

  py::class_<IdmController<T>, LeafSystem<T>>(
      m, "IdmController", doc.IdmController.doc)
      .def(py::init<const maliput::api::RoadGeometry&, ScanStrategy,
               RoadPositionStrategy, double>(),
          py::arg("road"), py::arg("path_or_branches"),
          py::arg("road_position_strategy"), py::arg("period_sec"),
          doc.IdmController.ctor.doc)
      .def("ego_pose_input", &IdmController<T>::ego_pose_input,
          py_reference_internal, doc.IdmController.ego_pose_input.doc)
      .def("ego_velocity_input", &IdmController<T>::ego_velocity_input,
          py_reference_internal, doc.IdmController.ego_velocity_input.doc)
      .def("traffic_input", &IdmController<T>::traffic_input,
          py_reference_internal, doc.IdmController.traffic_input.doc)
      .def("acceleration_output", &IdmController<T>::acceleration_output,
          py_reference_internal, doc.IdmController.acceleration_output.doc);

  py::class_<PoseSelector<T>>(m, "PoseSelector", doc.PoseSelector.doc)
      .def_static("FindClosestPair",
          [](const maliput::api::Lane* lane,
              const systems::rendering::PoseVector<T>& ego_pose,
              const systems::rendering::PoseBundle<T>& traffic_poses,
              const T& scan_distance, ScanStrategy path_or_branches) {
            return PoseSelector<T>::FindClosestPair(
                lane, ego_pose, traffic_poses, scan_distance, path_or_branches);
          },
          py::arg("lane"), py::arg("ego_pose"), py::arg("traffic_poses"),
          py::arg("scan_distance"), py::arg("path_or_branches"),
          doc.PoseSelector.FindClosestPair.doc)
      .def_static("FindSingleClosestPose",
          [](const maliput::api::Lane* lane,
              const systems::rendering::PoseVector<T>& ego_pose,
              const systems::rendering::PoseBundle<T>& traffic_poses,
              const T& scan_distance, const AheadOrBehind side,
              ScanStrategy path_or_branches) {
            return PoseSelector<T>::FindSingleClosestPose(lane, ego_pose,
                traffic_poses, scan_distance, side, path_or_branches);
          },
          py::arg("lane"), py::arg("ego_pose"), py::arg("traffic_poses"),
          py::arg("scan_distance"), py::arg("side"),
          py::arg("path_or_branches"),
          doc.PoseSelector.FindSingleClosestPose.doc)
      .def_static("GetSigmaVelocity", &PoseSelector<T>::GetSigmaVelocity,
          doc.PoseSelector.GetSigmaVelocity.doc);

  py::class_<PurePursuitController<T>, LeafSystem<T>>(
      m, "PurePursuitController", doc.PurePursuitController.doc)
      .def(py::init<>(), doc.PurePursuitController.ctor.doc)
      .def("ego_pose_input", &PurePursuitController<T>::ego_pose_input,
          py_reference_internal, doc.PurePursuitController.ego_pose_input.doc)
      .def("lane_input", &PurePursuitController<T>::lane_input,
          py_reference_internal, doc.PurePursuitController.lane_input.doc)
      .def("steering_command_output",
          &PurePursuitController<T>::steering_command_output,
          py_reference_internal,
          doc.PurePursuitController.steering_command_output.doc);

  // TODO(eric.cousineau) Bind this named vector automatically (see #8096).
  py::class_<SimpleCarState<T>, BasicVector<T>>(
      m, "SimpleCarState", doc.SimpleCarState.doc)
      .def(py::init<>(), doc.SimpleCarState.ctor.doc)
      .def("x", &SimpleCarState<T>::x, doc.SimpleCarState.x.doc)
      .def("y", &SimpleCarState<T>::y, doc.SimpleCarState.y.doc)
      .def("heading", &SimpleCarState<T>::heading,
          doc.SimpleCarState.heading.doc)
      .def("velocity", &SimpleCarState<T>::velocity,
          doc.SimpleCarState.velocity.doc)
      .def("set_x", &SimpleCarState<T>::set_x, doc.SimpleCarState.set_x.doc)
      .def("set_y", &SimpleCarState<T>::set_y, doc.SimpleCarState.set_y.doc)
      .def("set_heading", &SimpleCarState<T>::set_heading,
          doc.SimpleCarState.set_heading.doc)
      .def("set_velocity", &SimpleCarState<T>::set_velocity,
          doc.SimpleCarState.set_velocity.doc);

  py::class_<SimpleCar<T>, LeafSystem<T>>(m, "SimpleCar", doc.SimpleCar.doc)
      .def(py::init<>(), doc.SimpleCar.ctor.doc)
      .def("state_output", &SimpleCar<T>::state_output, py_reference_internal,
          doc.SimpleCar.state_output.doc)
      .def("pose_output", &SimpleCar<T>::pose_output, py_reference_internal,
          doc.SimpleCar.pose_output.doc)
      .def("velocity_output", &SimpleCar<T>::velocity_output,
          py_reference_internal, doc.SimpleCar.velocity_output.doc);

  // TODO(jadecastro) Bind more systems as appropriate.
}

}  // namespace pydrake
}  // namespace drake
