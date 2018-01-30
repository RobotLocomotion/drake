#pragma once

#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/systems/rendering/frame_velocity.h"
#include "drake/systems/rendering/pose_vector.h"

namespace drake {
namespace automotive {

/// Given a PoseVector @p pose, find a car's current RoadGeometry via a search
/// of immediate ongoing lanes, starting with the current one.  Uses the
/// provided FrameVelocity @p velocity to determine which side of the lane
/// (provided in @p rp) to check.  Updates @p rp with the result, if one is
/// found; otherwise updates @p rp with the default RoadPosition.
///
/// Instantiated templates for the following ScalarTypes are provided:
/// - double
/// - drake::AutoDiffXd
///
/// They are already available to link against in the containing library.
///
/// TODO(jadecastro) Enable symbolic::Expression (currently,
/// PoseSelector::GetSigmaVelocity() is blocking us, as it uses
/// ExtractDoubleOrThrow()).
template <typename T>
void CalcOngoingRoadPosition(
    const systems::rendering::PoseVector<T>& pose,
    const systems::rendering::FrameVelocity<T>& velocity,
    maliput::api::RoadPosition* rp);

}  // namespace automotive
}  // namespace drake
