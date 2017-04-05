#pragma once

#include <memory>

#include <Eigen/Geometry>

#include "drake/automotive/gen/driving_command.h"
#include "drake/automotive/gen/pure_pursuit_params.h"
#include "drake/automotive/gen/simple_car_params.h"
#include "drake/automotive/lane_direction.h"
#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/rendering/pose_vector.h"

namespace drake {
namespace automotive {

/// PurePursuitController implements a pure pursuit controller.  See PurePursuit
/// for details on the approach.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
///
/// They are already available to link against in the containing library.
///
/// Input Port 0: a DrivingCommand input.  Existing acceleration is passed
///   through; steering command is overwritten.
///   (InputPortDescriptor getter: driving_command_input())
/// Input Port 1: a LaneDirection representing the requested lane and direction
///   of travel.
///   (InputPortDescriptor getter: lane_input())
/// Input Port 2: PoseVector for the ego car.
///   (InputPortDescriptor getter: ego_pose_input())
///
/// Output Port 0: A DrivingCommand with the following elements:
///   * steering angle (overwrites input).
///   * acceleration (passes through from input).
///
/// @ingroup automotive_systems
template <typename T>
class PurePursuitController : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PurePursuitController)

  /// Constructor.
  explicit PurePursuitController(const maliput::api::RoadGeometry& road);
  ~PurePursuitController() override;

  /// Returns the port to the individual input/output ports.
  const systems::InputPortDescriptor<T>& driving_command_input() const;
  const systems::InputPortDescriptor<T>& lane_input() const;
  const systems::InputPortDescriptor<T>& ego_pose_input() const;
  const systems::OutputPortDescriptor<T>& driving_command_output() const;

 private:
  void DoCalcOutput(const systems::Context<T>& context,
                    systems::SystemOutput<T>* output) const override;

  void ImplDoCalcOutput(const PurePursuitParams<T>& pp_params,
                        const SimpleCarParams<T>& car_params,
                        const DrivingCommand<T>& input_command,
                        const LaneDirection& lane_direction,
                        const systems::rendering::PoseVector<T>& ego_pose,
                        DrivingCommand<T>* output_command) const;

  const maliput::api::RoadGeometry& road_;

  // Indices for the input / output ports.
  int command_input_index_{};
  int lane_index_{};
  int ego_pose_index_{};
  int command_output_index_{};
};

}  // namespace automotive
}  // namespace drake
