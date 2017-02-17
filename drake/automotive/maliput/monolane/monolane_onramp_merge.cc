#include "drake/automotive/maliput/monolane/monolane_onramp_merge.h"

#include "drake/automotive/maliput/monolane/road_geometry.h"

namespace drake {
namespace automotive {

namespace mono = maliput::monolane;

template <typename T>
std::unique_ptr<const maliput::api::RoadGeometry>
MonolaneOnrampMerge<T>::BuildOnramp() {
  // Initialize the road from the origin.
  const mono::EndpointXy& kOriginXy{0., 0., 0.};
  const mono::EndpointZ& kFlatZ{0., 0., 0., 0.};
  const mono::Endpoint& kRoadOrigin{kOriginXy, kFlatZ};

  // Construct the pre-merge road.
  const double& kPreArcLength = 25.;
  const double& kPreArcRadius = 40.;
  const auto& pre0 = rb_->Connect(
      "pre0", kRoadOrigin,
      mono::ArcOffset(kPreArcLength, -kPreArcRadius / kPreArcLength), kFlatZ);
  const auto& pre1 = rb_->Connect(
      "pre1", pre0->end(),
      mono::ArcOffset(kPreArcLength, kPreArcRadius / kPreArcLength), kFlatZ);
  const auto& pre2 = rb_->Connect(
      "pre2", pre1->end(),
      mono::ArcOffset(kPreArcLength, -kPreArcRadius / kPreArcLength), kFlatZ);
  const auto& pre3 = rb_->Connect(
      "pre3", pre2->end(),
      mono::ArcOffset(kPreArcLength, kPreArcRadius / kPreArcLength), kFlatZ);
  const auto& pre4 = rb_->Connect(
      "pre4", pre3->end(),
      mono::ArcOffset(kPreArcLength, -kPreArcRadius / kPreArcLength), kFlatZ);
  const auto& pre5 = rb_->Connect(
      "pre5", pre4->end(),
      mono::ArcOffset(kPreArcLength, kPreArcRadius / kPreArcLength), kFlatZ);

  // Construct the post-merge road.
  const double& kPostLinearLength = 100.;
  rb_->Connect("post0", pre5->end(), kPostLinearLength, kFlatZ);

  // Construct the on-ramp (starting at merge junction and working backwards).
  const double& kOnrampArcLength = 35.;
  const double& kOnrampArcRadius = 50.;
  const double& kOnrampLinearLength = 100.;
  const auto& onramp1 = rb_->Connect(
      "onramp1", pre5->end(),
      mono::ArcOffset(kOnrampArcLength, kOnrampArcRadius / kOnrampArcLength),
      kFlatZ);

  rb_->Connect("onramp0", onramp1->end(), kOnrampLinearLength, kFlatZ);

  return rb_->Build({"monolane-merge-example"});
}

template class MonolaneOnrampMerge<double>;
// TODO(jadecastro): Add explicit instantiations for `TaylorVarXd` and
// `symbolic::Expression` types.

}  // namespace automotive
}  // namespace drake
