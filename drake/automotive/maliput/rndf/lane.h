#pragma once

#include <cmath>
#include <memory>

#include <Eigen/Dense>

#include "drake/automotive/maliput/api/branch_point.h"
#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/segment.h"
#include "drake/automotive/maliput/rndf/branch_point.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/math/roll_pitch_yaw.h"

namespace drake {
namespace maliput {
namespace rndf {

class BranchPoint;

typedef Vector2<double> V2;
typedef Vector3<double> V3;

/// An R^3 rotation parameterized by roll, pitch, yaw.
///
/// This effects a compound rotation around space-fixed x-y-z axes:
///
///   Rot3(yaw,pitch,roll) * V = RotZ(yaw) * RotY(pitch) * RotX(roll) * V
class Rot3 {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Rot3)

  Rot3(double roll, double pitch, double yaw) : rpy_(roll, pitch, yaw) {}

  /// Applies the rotation to a 3-vector.
  V3 apply(const V3& in) const { return math::rpy2rotmat(rpy_) * in; }

  double yaw() const { return rpy_(2); }
  double pitch() const { return rpy_(1); }
  double roll() const { return rpy_(0); }

 private:
  Eigen::Matrix<double, 3, 1, Eigen::DontAlign> rpy_;
};


/// Base class for the RNDF implementation of api::Lane.
class Lane : public api::Lane {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Lane)

  /// Constructs a Lane.
  /// @param id is the ID of the api::Lane.
  /// @param segment is a pointer that refers to its parent, which must remain
  /// valid for the lifetime of this class.
  /// @param width is the value of the width of the lane based on the RNDF
  /// lane_width parameter. It will be used to set a constant lane_bound
  /// value along the road.
  /// @param index is the index that can be used to reference this Lane from
  /// api::Segment::lane() call.
  ///
  /// This is the base class for subclasses, each of which describe a
  /// primitive reference curve in the xy ground-plane of the world frame.
  /// The specific curve is expressed by a subclass's implementations of
  /// private virtual functions.
  ///
  /// This base implementation will handle all the non-geometric stuff from the
  /// lane. All geometric computation will be moved to each sub lane childs. See
  /// SplineLane for an example.
  Lane(const api::LaneId& id, const api::Segment* segment,
      const double width,
      const int index) :
        id_(id), segment_(segment),
        width_(width),
        index_(index) {
  }

  void SetStartBp(BranchPoint* bp) { start_bp_ = bp; }
  void SetEndBp(BranchPoint* bp) { end_bp_ = bp; }

  BranchPoint* start_bp() { return start_bp_; }

  BranchPoint* end_bp() { return end_bp_; }

  ~Lane() override = default;

 protected:
  const api::LaneId do_id() const override { return id_; }

  const api::Segment* do_segment() const override;

  int do_index() const override { return index_; }

  const api::Lane* do_to_left() const override {
    if ((segment_->num_lanes() - 1) == index_) {
      return nullptr;
    }
    return segment_->lane(index_ + 1);
  }

  const api::Lane* do_to_right() const override {
    if (index_ == 0) {
      return nullptr;
    }
    return segment_->lane(index_ - 1);
  }

  const api::BranchPoint* DoGetBranchPoint(
      const api::LaneEnd::Which which_end) const override;

  const api::LaneEndSet* DoGetConfluentBranches(
      const api::LaneEnd::Which which_end) const override;

  const api::LaneEndSet* DoGetOngoingBranches(
      const api::LaneEnd::Which which_end) const override;

  std::unique_ptr<api::LaneEnd> DoGetDefaultBranch(
      const api::LaneEnd::Which which_end) const override;

  const api::LaneId id_;
  const api::Segment* segment_{};
  BranchPoint* start_bp_{};
  BranchPoint* end_bp_{};
  const double width_{};
  const int index_{};
};


}  // namespace rndf
}  // namespace maliput
}  // namespace drake
