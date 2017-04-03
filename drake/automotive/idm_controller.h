#pragma once

#include <memory>

#include <Eigen/Geometry>

#include "drake/automotive/gen/driving_command.h"
#include "drake/automotive/gen/idm_planner_parameters.h"
#include "drake/automotive/idm_planner.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/pose_selector.h"
#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/rendering/pose_bundle.h"
#include "drake/systems/rendering/pose_vector.h"

namespace drake {
namespace automotive {

/// An IdmController implements the IDM (Intelligent Driver Model) planner,
/// computed based only on the nearest car ahead.  See IdmPlanner and
/// PoseSelector for details.  The output of this block is a DrivingCommand that
/// modifies the throttle and brake, but not the steering angle.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
///
/// They are already available to link against in the containing library.
///
/// Input Port 0: @p ego_pose PoseVector for the ego car.
///   (InputPortDescriptor getter: ego_pose_input())
/// Input Port 1: @p ego_velocity FrameVelocity of the ego car.
///   (InputPortDescriptor getter: ego_velocity_input())
/// Input Port 2: @p traffic_poses PoseBundle for the traffic cars, possibly
///   inclusive of the ego car's pose.
///   (InputPortDescriptor getter: traffic_input())
///
/// Output Port 0: A DrivingCommand with the following elements:
///   * steering angle (unused - outputs zero).
///   * acceleration.
///
/// @ingroup automotive_systems
template <typename T>
class IdmController : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(IdmController)

  /// Constructor.
  /// @param road is the pre-defined RoadGeometry.
  explicit IdmController(const maliput::api::RoadGeometry& road);
  ~IdmController() override;

  /// See the class description for details on the following input ports.
  /// @{
  const systems::InputPortDescriptor<T>& ego_pose_input() const;
  const systems::InputPortDescriptor<T>& ego_velocity_input() const;
  const systems::InputPortDescriptor<T>& traffic_input() const;
  /// @}

 private:
  // Converts @p pose into RoadPosition.
  const maliput::api::RoadPosition GetRoadPosition(
      const Isometry3<T>& pose) const;

  void DoCalcOutput(const systems::Context<T>& context,
                    systems::SystemOutput<T>* output) const override;

  void ImplDoCalcOutput(
      const systems::rendering::PoseVector<T>& ego_pose,
      const systems::rendering::FrameVelocity<T>& ego_velocity,
      const systems::rendering::PoseBundle<T>& traffic_poses,
      const IdmPlannerParameters<T>& idm_params,
      DrivingCommand<T>* output) const;

  const maliput::api::RoadGeometry& road_;

  // Indices for the input ports.
  int ego_pose_index_{};
  int ego_velocity_index_{};
  int traffic_index_{};
};

}  // namespace automotive
}  // namespace drake
