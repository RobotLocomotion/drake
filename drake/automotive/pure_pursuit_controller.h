#pragma once

#include <memory>

#include <Eigen/Geometry>

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
/// Input Port 0: a LaneDirection representing the requested lane and direction
///   of travel.
///   (InputPortDescriptor getter: lane_input())
/// Input Port 1: PoseVector for the ego car.
///   (InputPortDescriptor getter: ego_pose_input())
///
/// Output Port 0: A singleton BasicVector with the commanded steering angle.
///   (InputPortDescriptor getter: steering_command_output())
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
  const systems::InputPortDescriptor<T>& lane_input() const;
  const systems::InputPortDescriptor<T>& ego_pose_input() const;
  const systems::OutputPortDescriptor<T>& steering_command_output() const;

 private:
  void DoCalcOutput(const systems::Context<T>& context,
                    systems::SystemOutput<T>* output) const override;

  void ImplDoCalcOutput(const PurePursuitParams<T>& pp_params,
                        const SimpleCarParams<T>& car_params,
                        const LaneDirection& lane_direction,
                        const systems::rendering::PoseVector<T>& ego_pose,
                        systems::BasicVector<T>* command) const;

  const maliput::api::RoadGeometry& road_;

  // Indices for the input / output ports.
  const int lane_index_{};
  const int ego_pose_index_{};
  const int steering_command_index_{};
};

}  // namespace automotive
}  // namespace drake
