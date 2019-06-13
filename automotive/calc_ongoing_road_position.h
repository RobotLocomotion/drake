#pragma once

#include "drake/automotive/deprecated.h"
#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/systems/rendering/frame_velocity.h"
#include "drake/systems/rendering/pose_vector.h"

namespace drake {
namespace automotive {

/// Given a PoseVector @p pose, find a car's current RoadGeometry via a search
/// of immediate ongoing lanes, starting with the current one.  Uses the
/// provided FrameVelocity @p velocity to determine which side of the lane
/// (provided in @p rp) to check.  Updates @p rp with the result, if one is
/// found; otherwise updates @p rp using the result of a global search of the @p
/// road.
///
/// Instantiated templates for the following scalar types `T` are provided:
///
/// - double
/// - drake::AutoDiffXd
///
/// They are already available to link against in the containing library.
//
// TODO(jadecastro) Enable symbolic::Expression (currently,
// PoseSelector::GetSigmaVelocity() is blocking us, as it uses
// ExtractDoubleOrThrow()).
template <typename T>
void CalcOngoingRoadPosition(
    const systems::rendering::PoseVector<T>& pose,
    const systems::rendering::FrameVelocity<T>& velocity,
    const maliput::api::RoadGeometry& road,
    maliput::api::RoadPosition* rp);

}  // namespace automotive
}  // namespace drake
