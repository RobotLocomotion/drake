#include "drake/automotive/monolane_onramp_merge.h"

#include <cmath>
#include <iostream>
#include <memory>
#include <utility>

#include <Eigen/Geometry>

#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/monolane/builder.h"

namespace drake {
namespace automotive {

namespace mono = maliput::monolane;

template <typename T>
void MonolaneOnrampMerge<T>::BuildOnramp() {
  mono::Builder b{road_.lane_bounds, road_.driveable_bounds, kLinearTolerance_,
                  kAngularTolerance_};

  // Construct the pre-merge road.
  std::unique_ptr<mono::RoadSectionBuilder<T>> rs_pre(
      new mono::RoadSectionBuilder<T>(std::move(b_)));
  rs_pre->AddArcPrimitive(40., 25., mono::kCW);
  rs_pre->AddArcPrimitive(20., 25., mono::kCCW);
  rs_pre->AddArcPrimitive(40., 25., mono::kCW);
  rs_pre->AddArcPrimitive(20., 25., mono::kCCW);
  rs_pre->AddArcPrimitive(40., 25., mono::kCW);
  rs_pre->AddArcPrimitive(20., 25., mono::kCCW);
  const mono::Endpoint endpoint_pre = rs_pre->get_last_endpoint();
  b_ = rs_pre->Finalize();

  // Construct the post-merge road.
  std::unique_ptr<mono::RoadSectionBuilder<T>> rs_post(
      new mono::RoadSectionBuilder<T>(std::move(b_), endpoint_pre));
  rs_post->AddLinearPrimitive(50.);
  b_ = rs_post->Finalize();

  // Construct the on-ramp.
  std::unique_ptr<mono::RoadSectionBuilder<T>> rs_onramp(
      new mono::RoadSectionBuilder<T>(std::move(b_), endpoint_pre.reverse()));
  rs_onramp->AddArcPrimitive(50., 30, mono::kCCW);
  rs_onramp->AddLinearPrimitive(100.);
  b_ = rs_onramp->Finalize();

  rg_ = b_->Build({"monolane-merge-example"});
}

template class MonolaneOnrampMerge<double>;
// TODO(jadecastro): Bring instantiations online for `TaylorVarXd>` and
// `symbolic::Expression` types.

}  // namespace automotive
}  // namespace drake
