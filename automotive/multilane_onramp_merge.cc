#include "drake/automotive/multilane_onramp_merge.h"

#include "drake/automotive/maliput/api/road_geometry.h"

namespace drake {
namespace automotive {

using maliput::api::LaneEnd;

namespace multi = maliput::multilane;

std::unique_ptr<const maliput::api::RoadGeometry>
MultilaneOnrampMerge::BuildOnramp() const {
  auto rb = multi::BuilderFactory().Make(rc_.lane_width, rc_.elevation_bounds,
                                         linear_tolerance_, angular_tolerance_,
                                         scale_length_, computation_policy_);

  // Initialize roads lane layouts.
  // Reference lane from which the reference curve of the segment is placed (at
  // kRefR0 lateral distance).
  const int kRefLane = 0;
  // Distance between the reference curve and kRefLane lane curve.
  const double kRefR0 = 0.;
  const multi::LaneLayout lane_layout(rc_.left_shoulder, rc_.right_shoulder,
                                      rc_.lane_number, kRefLane, kRefR0);

  // Initialize the road from the origin.
  const multi::EndpointXy kOriginXy{0., 0., 0.};
  const multi::EndpointZ kFlatZ{0., 0., 0., {}};
  const multi::Endpoint kRoadOrigin{kOriginXy, kFlatZ};

  // Construct the post-merge road.
  const double kPostArcLength = 25.;
  const double kPostArcRadius = 40.;
  const auto& post5 = rb->Connect(
      "post5", lane_layout,
      multi::StartReference().at(kRoadOrigin, multi::Direction::kForward),
      multi::ArcOffset(kPostArcLength, -kPostArcRadius / kPostArcLength),
      multi::EndReference().z_at(kFlatZ, multi::Direction::kForward));
  const auto& post4 = rb->Connect(
      "post4", lane_layout,
      multi::StartReference().at(*post5, LaneEnd::kFinish,
                                 multi::Direction::kForward),
      multi::ArcOffset(kPostArcLength, kPostArcRadius / kPostArcLength),
      multi::EndReference().z_at(kFlatZ, multi::Direction::kForward));
  const auto& post3 = rb->Connect(
      "post3", lane_layout,
      multi::StartReference().at(*post4, LaneEnd::kFinish,
                                 multi::Direction::kForward),
      multi::ArcOffset(kPostArcLength, -kPostArcRadius / kPostArcLength),
      multi::EndReference().z_at(kFlatZ, multi::Direction::kForward));
  const auto& post2 = rb->Connect(
      "post2", lane_layout,
      multi::StartReference().at(*post3, LaneEnd::kFinish,
                                 multi::Direction::kForward),
      multi::ArcOffset(kPostArcLength, kPostArcRadius / kPostArcLength),
      multi::EndReference().z_at(kFlatZ, multi::Direction::kForward));
  const auto& post1 = rb->Connect(
      "post1", lane_layout,
      multi::StartReference().at(*post2, LaneEnd::kFinish,
                                 multi::Direction::kForward),
      multi::ArcOffset(kPostArcLength, -kPostArcRadius / kPostArcLength),
      multi::EndReference().z_at(kFlatZ, multi::Direction::kForward));
  const auto& post0 = rb->Connect(
      "post0", lane_layout,
      multi::StartReference().at(*post1, LaneEnd::kFinish,
                                 multi::Direction::kForward),
      multi::ArcOffset(kPostArcLength, kPostArcRadius / kPostArcLength),
      multi::EndReference().z_at(kFlatZ, multi::Direction::kForward));

  // Construct the pre-merge road.
  const double kPostLinearLength = 100.;
  const int pre_num_lanes = rc_.lane_number / 2 + 1;
  const multi::LaneLayout pre_lane_layout(rc_.left_shoulder, rc_.right_shoulder,
                                          pre_num_lanes, kRefLane, kRefR0);
  const auto& pre0 = rb->Connect(
      "pre0", pre_lane_layout,
      multi::StartReference().at(*post0, LaneEnd::kFinish,
                                 multi::Direction::kForward),
      multi::LineOffset(kPostLinearLength),
      multi::EndReference().z_at(kFlatZ, multi::Direction::kForward));

  // Construct the on-ramp (starting at merge junction at the `pre0` - `post0`
  // interface and working backwards).
  const double kOnrampArcLength = 35.;
  const double kOnrampArcRadius = 50.;
  const double kOnrampLinearLength = 100.;
  const int onramp_num_lanes = rc_.lane_number / 2 + 1;
  const multi::LaneLayout on_ramp_lane_layout(
      rc_.left_shoulder, rc_.right_shoulder, onramp_num_lanes, kRefLane,
      kRefR0 +
          static_cast<double>(rc_.lane_number - onramp_num_lanes) *
              rc_.lane_width);
  const auto& onramp1 = rb->Connect(
      "onramp1", on_ramp_lane_layout,
      multi::StartReference().at(*post0, LaneEnd::kFinish,
                                 multi::Direction::kForward),
      multi::ArcOffset(kOnrampArcLength, kOnrampArcRadius / kOnrampArcLength),
      multi::EndReference().z_at(kFlatZ, multi::Direction::kForward));
  const auto& onramp0 = rb->Connect(
      "onramp0", on_ramp_lane_layout,
      multi::StartReference().at(*onramp1, LaneEnd::kFinish,
                                 multi::Direction::kForward),
      multi::LineOffset(kOnrampLinearLength),
      multi::EndReference().z_at(kFlatZ, multi::Direction::kForward));

  // Manually specify the default branches for all junctions in the road.
  auto set_default_branches = [&](
      const multi::Connection* in, int in_first_lane,
      const multi::Connection* out, int out_first_lane, int num_lanes) {
    for (int i = 0; i < num_lanes; ++i) {
      rb->SetDefaultBranch(in, i + in_first_lane, LaneEnd::kStart, out,
                           i + out_first_lane, LaneEnd::kFinish);
    }
  };
  set_default_branches(pre0, 0, post0, 0, pre_num_lanes);
  set_default_branches(post0, 0, post1, 0, rc_.lane_number);
  set_default_branches(post1, 0, post2, 0, rc_.lane_number);
  set_default_branches(post2, 0, post3, 0, rc_.lane_number);
  set_default_branches(post3, 0, post4, 0, rc_.lane_number);
  set_default_branches(post4, 0, post5, 0, rc_.lane_number);
  set_default_branches(onramp1, 0, post0,
                       rc_.lane_number / 2 - (rc_.lane_number % 2 == 0 ? 1 : 0),
                       onramp_num_lanes);
  set_default_branches(onramp0, 0, onramp1, 0, onramp_num_lanes);

  return rb->Build(maliput::api::RoadGeometryId{"multilane-merge-example"});
}

}  // namespace automotive
}  // namespace drake
