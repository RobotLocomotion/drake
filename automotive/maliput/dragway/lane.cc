#include "drake/automotive/maliput/dragway/lane.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include "drake/automotive/maliput/dragway/branch_point.h"
#include "drake/automotive/maliput/dragway/road_geometry.h"
#include "drake/automotive/maliput/dragway/segment.h"
#include "drake/common/drake_assert.h"
#include "drake/math/saturate.h"

using std::make_unique;

namespace drake {
namespace maliput {
namespace dragway {

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
      api::BranchPointId(id.string() + "_Branch_Point"), this,
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

optional<api::LaneEnd> Lane::DoGetDefaultBranch(
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

api::GeoPositionT<AutoDiffXd> Lane::DoToGeoPositionAutoDiff(
    const api::LanePositionT<AutoDiffXd>& lane_pos) const {
  return {lane_pos.s(),
          lane_pos.r() + AutoDiffXd(Lane::y_offset()),
          lane_pos.h()};
}

api::Rotation Lane::DoGetOrientation(
    const api::LanePosition&) const {
  return api::Rotation();  // Default is Identity.
}

api::LanePosition Lane::DoToLanePosition(
    const api::GeoPosition& geo_pos,
    api::GeoPosition* nearest_point,
    double* distance) const {
  return ImplDoToLanePositionT<double>(geo_pos, nearest_point, distance);
}

api::LanePositionT<AutoDiffXd> Lane::DoToLanePositionAutoDiff(
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
  using math::saturate;

  const T min_x{0.};
  const T max_x{length_};
  const T min_y{driveable_bounds_.min() + y_offset_};
  const T max_y{driveable_bounds_.max() + y_offset_};
  const T min_z{elevation_bounds_.min()};
  const T max_z{elevation_bounds_.max()};

  const T x = geo_pos.x();
  const T y = geo_pos.y();
  const T z = geo_pos.z();

  api::GeoPositionT<T> closest_point{
    saturate(x, min_x, max_x),
    saturate(y, min_y, max_y),
    saturate(z, min_z, max_z)};
  if (nearest_point != nullptr) {
    *nearest_point = closest_point;
  }

  if (distance != nullptr) {
    const T distance_unsat = (geo_pos.xyz() - closest_point.xyz()).norm();

    // N.B. Under AutoDiff, the partial derivative of the distance with respect
    // to position is undefined (i.e. NaN) when distance.value() = 0.  This
    // implementation replaces those NaN values with numbers that are consistent
    // with the geometry such that the following hold:
    //
    // Let v be any coordinate x, y, or z.
    //
    // 1) Within the interior of the lane volume, ∂/∂v(distance) = 0, since
    // distance is invariant to perturbations in v.
    //
    // 2) On the exterior of the lane, ∂/∂v(distance) is identical to
    // ∂/∂v(distance_unsat).
    //
    // 3) On the boundary, ∂/∂v(distance) has two solutions: zero or
    // ∂/∂v(distance_unsat), depending on whether the derivative at the boundary
    // is evaluated when approached from the exterior or interior.  This
    // implementation chooses the derivatives taken from within the interior
    // (zero) in order to remain consistent with the derivatives of
    // nearest_point and the returned LanePositionT.
    if (distance_unsat > T(0.)) {
      *distance = distance_unsat;
    } else {  // ∂/∂x(distance) = 0 when distance = 0.
      *distance = T(0.);
    }
  }

  return {closest_point.x(),
          closest_point.y() - T(y_offset_),
          closest_point.z()};
}

}  // namespace dragway
}  // namespace maliput
}  // namespace drake
