#include "drake/automotive/monolane_onramp_merge.h"

#include <cmath>
#include <iostream>

#include <Eigen/Geometry>

#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/monolane/builder.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/symbolic_expression.h"

namespace drake {
namespace automotive {

namespace mono = maliput::monolane;

template <typename T>
void MonolaneOnrampMerge<T>::BuildOnramp() {
  mono::Builder b{road_.lane_bounds, road_.driveable_bounds, kLinearTolerance_,
        kAngularTolerance_};

  // Construct the pre-merge road.
  std::unique_ptr<mono::RoadSectionBuilder<T>> rs_pre(
      new mono::RoadSectionBuilder<T>(std::move(b_), false));
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
      new mono::RoadSectionBuilder<T>(std::move(b_), false, endpoint_pre));
  rs_post->AddLinearPrimitive(50.);
  b_ = rs_post->Finalize();

  // Construct the on-ramp.
  std::unique_ptr<mono::RoadSectionBuilder<T>> rs_onramp(
      new mono::RoadSectionBuilder<T>(std::move(b_), true,
                                      endpoint_pre.reverse()));
  rs_onramp->AddArcPrimitive(50., 30, mono::kCCW);
  rs_onramp->AddLinearPrimitive(100.);
  b_ = rs_onramp->Finalize();

  rg_ = b_->Build({"monolane-merge-example"});
}

template class MonolaneOnrampMerge<double>;
template class MonolaneOnrampMerge<TaylorVarXd>;
template class MonolaneOnrampMerge<symbolic::Expression>;

}  // namespace automotive
}  // namespace drake
