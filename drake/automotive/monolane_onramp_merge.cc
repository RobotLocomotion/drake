#include "drake/automotive/monolane_onramp_merge.h"

#include <cmath>
#include <iostream>

#include <Eigen/Geometry>

#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/monolane/builder.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/symbolic_expression.h"

namespace drake {
namespace maliput {
namespace monolane {

template <typename T>
void MonolaneOnrampMerge<T>::BuildOnramp() {
  Builder b{road_.lane_bounds, road_.driveable_bounds, kLinearTolerance_,
            kAngularTolerance_};

  // Construct the pre-merge road.
  std::unique_ptr<RoadSectionBuilder<T>> rs_pre(
      new RoadSectionBuilder<T>(std::move(b_), false));
  rs_pre->AddArcPrimitive(40., 25., kCW);
  rs_pre->AddArcPrimitive(20., 25., kCCW);
  rs_pre->AddArcPrimitive(40., 25., kCW);
  rs_pre->AddArcPrimitive(20., 25., kCCW);
  rs_pre->AddArcPrimitive(40., 25., kCW);
  rs_pre->AddArcPrimitive(20., 25., kCCW);
  const Endpoint endpoint_pre = rs_pre->get_last_endpoint();
  b_ = rs_pre->Finalize();

  // Construct the post-merge road.
  std::unique_ptr<RoadSectionBuilder<T>> rs_post(
      new RoadSectionBuilder<T>(std::move(b_), false, endpoint_pre));
  rs_post->AddLinearPrimitive(50.);
  b_ = rs_post->Finalize();

  // Construct the on-ramp.
  std::unique_ptr<RoadSectionBuilder<T>> rs_onramp(new RoadSectionBuilder<T>(
      std::move(b_), true, endpoint_pre.reverse()));
  rs_onramp->AddArcPrimitive(50., 30, kCCW);
  rs_onramp->AddLinearPrimitive(100.);
  b_ = rs_onramp->Finalize();

  // b_.SetDefaultBranch(
  //    left1, api::LaneEnd::kStart, left1, api::LaneEnd::kFinish);

  std::cerr << " Building... " << std::endl;
  rg_ = b_->Build({"monolane-onramp-example"});
}

// These instantiations must match the API documentation in
// monolane_onramp_merge.h.
template class MonolaneOnrampMerge<double>;
template class MonolaneOnrampMerge<TaylorVarXd>;
template class MonolaneOnrampMerge<symbolic::Expression>;

}  // namespace monolane
}  // namespace maliput
}  // namespace drake
