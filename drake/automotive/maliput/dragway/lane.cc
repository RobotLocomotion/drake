#include "drake/automotive/maliput/dragway/lane.h"

#include <algorithm>
#include <cmath>
#include <memory>

#include "drake/automotive/maliput/dragway/branch_point.h"
#include "drake/automotive/maliput/dragway/road_geometry.h"
#include "drake/automotive/maliput/dragway/segment.h"
#include "drake/common/cond.h"
#include "drake/common/drake_assert.h"
#include "drake/math/saturate.h"

using std::make_unique;

namespace drake {
namespace maliput {
namespace dragway {

// Pad the partial derivatives of an AutoDiffXd variable `x` with a zero vector
// of the same dimension as `model_value`.  The `result` has the same value as
// `x`.
static AutoDiffXd PadZeroPartials(const AutoDiffXd& x,
                                  const AutoDiffXd& model_value) {
  DRAKE_DEMAND(x.derivatives().size() == 0);
  AutoDiffXd result(x);
  result.derivatives().resize(model_value.derivatives().size());
  result.derivatives().setZero();
  return result;
}

// No-op overload of SetZeroPartials when the arguments are of type double.
static double PadZeroPartials(const double& x, const double&) { return x; }

Lane::Lane(const Segment* segment, const api::LaneId& id,  int index,
    double length, double y_offset, const api::RBounds& lane_bounds,
    const api::RBounds& driveable_bounds,
    const api::HBounds& elevation_bounds)
    : segment_(segment),
      id_(id),
      index_(index),
      length_(length),
      y_offset_(y_offset),
      lane_bounds_(lane_bounds),
      driveable_bounds_(driveable_bounds),
      elevation_bounds_(elevation_bounds) {
  DRAKE_DEMAND(segment != nullptr);
  DRAKE_DEMAND(lane_bounds_.min() >= driveable_bounds_.min());
  DRAKE_DEMAND(lane_bounds_.max() <= driveable_bounds_.max());
  // TODO(liang.fok) Consider initializing this variable in the constructor's
  // initializer list so branch_point_ can be declared `const`.
  branch_point_ = make_unique<BranchPoint>(
      api::BranchPointId({id.id + "_Branch_Point"}), this,
      segment->junction()->road_geometry());
}

const api::Segment* Lane::do_segment() const {
  return segment_;
}

void Lane::set_lane_to_left(api::Lane* lane_to_left) {
  lane_to_left_ = lane_to_left;
}

void Lane::set_lane_to_right(api::Lane* lane_to_right) {
  lane_to_right_ = lane_to_right;
}

const api::BranchPoint* Lane::DoGetBranchPoint(
    const api::LaneEnd::Which) const {
  return branch_point_.get();
}

const api::LaneEndSet* Lane::DoGetConfluentBranches(
    api::LaneEnd::Which which_end) const {
  return branch_point_->GetConfluentBranches({this, which_end});
}

const api::LaneEndSet* Lane::DoGetOngoingBranches(
    api::LaneEnd::Which which_end) const {
  return branch_point_->GetOngoingBranches({this, which_end});
}

std::unique_ptr<api::LaneEnd> Lane::DoGetDefaultBranch(
    api::LaneEnd::Which which_end) const {
  return branch_point_->GetDefaultBranch({this, which_end});
}

api::RBounds Lane::do_lane_bounds(double) const {
  return lane_bounds_;
}

api::RBounds Lane::do_driveable_bounds(double) const {
  return driveable_bounds_;
}

api::HBounds Lane::do_elevation_bounds(double, double) const {
  return elevation_bounds_;
}

api::LanePosition Lane::DoEvalMotionDerivatives(
    const api::LanePosition&,
    const api::IsoLaneVelocity& velocity) const {
  return api::LanePosition(velocity.sigma_v, velocity.rho_v, velocity.eta_v);
}

api::GeoPosition Lane::DoToGeoPosition(
    const api::LanePosition& lane_pos) const {
  return {lane_pos.s(), lane_pos.r() + Lane::y_offset(), lane_pos.h()};
}

api::Rotation Lane::DoGetOrientation(
    const api::LanePosition&) const {
  return api::Rotation();  // Default is Identity.
}

api::LanePosition Lane::DoToLanePosition(
    const api::GeoPosition& geo_pos,
    api::GeoPosition* nearest_point,
    double* distance) const {
  auto geo_pos_double =
      dynamic_cast<const api::GeoPositionT<double>&>(geo_pos);
  auto nearest_point_double =
      dynamic_cast<api::GeoPositionT<double>*>(nearest_point);
  auto result = ImplDoToLanePositionT<double>(geo_pos_double,
                                              nearest_point_double, distance);
  return api::LanePosition::FromSrh({result.s(), result.r(), result.h()});
}

api::LanePositionT<double> Lane::DoToLanePositionT(
    const api::GeoPositionT<double>& geo_pos,
    api::GeoPositionT<double>* nearest_point,
    double* distance) const {
  return ImplDoToLanePositionT<double>(geo_pos, nearest_point, distance);
}

api::LanePositionT<AutoDiffXd> Lane::DoToLanePositionT(
    const api::GeoPositionT<AutoDiffXd>& geo_pos,
    api::GeoPositionT<AutoDiffXd>* nearest_point,
    AutoDiffXd* distance) const {
  return ImplDoToLanePositionT<AutoDiffXd>(geo_pos, nearest_point, distance);
}

template <typename T>
api::LanePositionT<T> Lane::ImplDoToLanePositionT(
    const api::GeoPositionT<T>& geo_pos,
    api::GeoPositionT<T>* nearest_point,
    T* distance) const {
  using std::max;

  const T min_x = PadZeroPartials(T(0.), geo_pos.x());
  const T max_x = PadZeroPartials(T(length_), geo_pos.x());
  const T min_y =
      PadZeroPartials(T(driveable_bounds_.min() + y_offset_), geo_pos.y());
  const T max_y =
      PadZeroPartials(T(driveable_bounds_.max() + y_offset_), geo_pos.y());
  const T min_z = PadZeroPartials(T(elevation_bounds_.min()), geo_pos.z());
  const T max_z = PadZeroPartials(T(elevation_bounds_.max()), geo_pos.z());

  api::GeoPositionT<T> closest_point{};
  closest_point.set_x(math::saturate(geo_pos.x(), min_x, max_x));
  closest_point.set_y(math::saturate(geo_pos.y(), min_y, max_y));
  closest_point.set_z(math::saturate(geo_pos.z(), min_z, max_z));
  if (nearest_point != nullptr) {
    *nearest_point = closest_point;
  }

  if (distance != nullptr) {
    T temp_distance = (geo_pos.xyz() - closest_point.xyz()).norm();
    *distance = cond(temp_distance == 0., PadZeroPartials(T(0.), *distance),
                     temp_distance);
  }

  api::LanePositionT<T> result{};
  result.set_s(closest_point.x());
  result.set_r(closest_point.y() - T(y_offset_));
  result.set_h(closest_point.z());
  return result;
}

}  // namespace dragway
}  // namespace maliput
}  // namespace drake
