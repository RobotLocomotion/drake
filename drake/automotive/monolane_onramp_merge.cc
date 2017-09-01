#include "drake/automotive/monolane_onramp_merge.h"

#include "drake/automotive/maliput/api/road_geometry.h"

namespace drake {
namespace automotive {

using maliput::api::LaneEnd;

namespace mono = maliput::monolane;

std::unique_ptr<const maliput::api::RoadGeometry>
MonolaneOnrampMerge::BuildOnramp() {
  std::unique_ptr<maliput::monolane::Builder> rb(
      new maliput::monolane::Builder(rc_.lane_bounds, rc_.driveable_bounds,
                                     rc_.elevation_bounds,
                                     linear_tolerance_, angular_tolerance_));

  // Initialize the road from the origin.
  const mono::EndpointXy kOriginXy{0., 0., 0.};
  const mono::EndpointZ kFlatZ{0., 0., 0., 0.};
  const mono::Endpoint kRoadOrigin{kOriginXy, kFlatZ};

  // Construct the post-merge road.
  const double kPostArcLength = 25.;
  const double kPostArcRadius = 40.;
  const auto& post5 = rb->Connect(
      "post5", kRoadOrigin,
      mono::ArcOffset(kPostArcLength, -kPostArcRadius / kPostArcLength),
      kFlatZ);
  const auto& post4 = rb->Connect(
      "post4", post5->end(),
      mono::ArcOffset(kPostArcLength, kPostArcRadius / kPostArcLength), kFlatZ);
  const auto& post3 = rb->Connect(
      "post3", post4->end(),
      mono::ArcOffset(kPostArcLength, -kPostArcRadius / kPostArcLength),
      kFlatZ);
  const auto& post2 = rb->Connect(
      "post2", post3->end(),
      mono::ArcOffset(kPostArcLength, kPostArcRadius / kPostArcLength), kFlatZ);
  const auto& post1 = rb->Connect(
      "post1", post2->end(),
      mono::ArcOffset(kPostArcLength, -kPostArcRadius / kPostArcLength),
      kFlatZ);
  const auto& post0 = rb->Connect(
      "post0", post1->end(),
      mono::ArcOffset(kPostArcLength, kPostArcRadius / kPostArcLength), kFlatZ);

  // Construct the pre-merge road.
  const double& kPostLinearLength = 100.;
  const auto& pre0 =
      rb->Connect("pre0", post0->end(), kPostLinearLength, kFlatZ);

  // Construct the on-ramp (starting at merge junction at the `pre0` - `post0`
  // interface and working backwards).
  const double& kOnrampArcLength = 35.;
  const double& kOnrampArcRadius = 50.;
  const double& kOnrampLinearLength = 100.;
  const auto& onramp1 = rb->Connect(
      "onramp1", post0->end(),
      mono::ArcOffset(kOnrampArcLength, kOnrampArcRadius / kOnrampArcLength),
      kFlatZ);
  const auto& onramp0 =
      rb->Connect("onramp0", onramp1->end(), kOnrampLinearLength, kFlatZ);

  // Manually specify the default branches for all junctions in the road.
  rb->SetDefaultBranch(pre0, LaneEnd::kStart, post0, LaneEnd::kFinish);
  rb->SetDefaultBranch(post0, LaneEnd::kStart, post1, LaneEnd::kFinish);
  rb->SetDefaultBranch(post1, LaneEnd::kStart, post2, LaneEnd::kFinish);
  rb->SetDefaultBranch(post2, LaneEnd::kStart, post3, LaneEnd::kFinish);
  rb->SetDefaultBranch(post3, LaneEnd::kStart, post4, LaneEnd::kFinish);
  rb->SetDefaultBranch(post4, LaneEnd::kStart, post5, LaneEnd::kFinish);
  rb->SetDefaultBranch(onramp1, LaneEnd::kStart, post0, LaneEnd::kFinish);
  rb->SetDefaultBranch(onramp0, LaneEnd::kStart, onramp1, LaneEnd::kFinish);

  return rb->Build(maliput::api::RoadGeometryId{"monolane-merge-example"});
}

}  // namespace automotive
}  // namespace drake
