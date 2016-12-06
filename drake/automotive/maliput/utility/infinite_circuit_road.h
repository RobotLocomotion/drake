#pragma once

#include <cmath>
#include <memory>
#include <utility>
#include <vector>

#include "drake/automotive/maliput/api/branch_point.h"
#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/automotive/maliput/api/junction.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/api/segment.h"

#include "drake/common/drake_assert.h"

namespace drake {
namespace maliput {
namespace utility {

/// A toy adapter class that makes a RoadGeometry look like it has a single
/// inifinitely long Lane.  Its primary (and perhaps only) purpose is to
/// facilitate demos until proper hybrid system support is available.
///
/// Caveats:
///  * Only works with a RoadGeometry that has one-lane-per-segment.
///  * Source RoadGeometry must have no dead-ends.
class InfiniteCircuitRoad : public api::RoadGeometry {
 private:
  class Lane;

 public:
  struct Record {
    const api::Lane* lane{};
    double start_circuit_s{};
    double end_circuit_s{};
    bool is_reversed{};
  };

  /// Construct an InfiniteCircuitRoad based on @param source, using
  /// @param start as the starting point in the search for a closed circuit.
  ///
  /// NB:  All the real construction work happens in the constructor for
  /// InfiniteCircuitRoad::Lane.
  InfiniteCircuitRoad(const api::RoadGeometryId& id,
                      const api::RoadGeometry* source,
                      const api::LaneEnd& start,
                      const std::vector<const api::Lane*>& path);

  virtual ~InfiniteCircuitRoad();

  /// @returns the sole Lane component emulated by this RoadGeometry.
  const Lane* lane() const { return &lane_; }

  /// @returns the actual length of a single cycle (despite the illusion that
  /// the road is infinitely long).
  double cycle_length() const { return lane_.cycle_length(); }

  /// @returns a pointer to the underlying source RoadGeometry.
  const api::RoadGeometry* source() const { return source_; }

  int num_path_records() const { return lane_.num_path_records(); }

  const Record path_record(int i) const { return lane_.path_record(i); }

  int GetPathIndex(double s) const { return lane_.GetPathIndex(s); }

  /// Project the given LanePosition @param lane_pos on the "infinite Lane"
  /// back to a RoadPosition in the source RoadGeometry.
  ///
  /// @returns a pair of:
  /// - RoadPosition indicating the source Lane and position on that Lane;
  /// - bool indicating travel should be reversed in the source Lane with
  ///   respect to +s travel in the Infinite Lane --- e.g., if true, then
  ///   +s motion in the Infinite Lane corresponds to -s motion in the
  ///   source Lane.
  std::pair<api::RoadPosition, bool> ProjectToSourceRoad(
      const api::LanePosition& lane_pos) const {
    return lane_.ProjectToSourceRoad(lane_pos);
  }

 private:
  class Junction;
  class Segment;

  class Lane : public api::Lane {
   public:
    Lane(const api::LaneId& id, const Segment* segment,
         const api::RoadGeometry* source,
         const api::LaneEnd& start,
         const std::vector<const api::Lane*>& path);

    virtual ~Lane();

    double cycle_length() const { return cycle_length_; }

    /// @returns the position within the fixed-length circuit of the given
    /// longitudinal position in the emulated 'infinite lane'.
    double circuit_s(const double s) const {
      // TODO(maddog@tri.global)  Yes, this has precision problems as
      //                          lane_pos.s_ grows without bound.
      double result = std::fmod(s, cycle_length_);
      return (result < 0.) ? (result + cycle_length_) : result;
    }

    int num_path_records() const { return records_.size(); }

    const Record path_record(int i) const { return records_[i]; }

    int GetPathIndex(double s) const;

    std::pair<api::RoadPosition, bool> ProjectToSourceRoad(
        const api::LanePosition& lane_pos) const;

   private:
    const api::LaneId do_id() const override { return id_; }

    const api::Segment* do_segment() const override { return segment_; }

    int do_index() const override { return 0; }  // Only one lane per segment!

    const api::Lane* do_to_left() const override { return nullptr; }

    const api::Lane* do_to_right() const override { return nullptr; }

    const api::BranchPoint* DoGetBranchPoint(
        const api::LaneEnd::Which which_end) const override {
      DRAKE_ABORT();
    }

    const api::LaneEndSet* DoGetConfluentBranches(
        const api::LaneEnd::Which which_end) const override {
      DRAKE_ABORT();
    }

    const api::LaneEndSet* DoGetOngoingBranches(
        const api::LaneEnd::Which which_end) const override {
      DRAKE_ABORT();
    }

    std::unique_ptr<api::LaneEnd> DoGetDefaultBranch(
        const api::LaneEnd::Which which_end) const override {
      DRAKE_ABORT();
    }

    double do_length() const override;

    api::RBounds do_lane_bounds(double) const override { return lane_bounds_; }

    api::RBounds do_driveable_bounds(double) const override {
      return driveable_bounds_;
    }

    api::GeoPosition DoToGeoPosition(
        const api::LanePosition& lane_pos) const override;

    api::Rotation DoGetOrientation(
        const api::LanePosition& lane_pos) const override;

    api::LanePosition DoEvalMotionDerivatives(
        const api::LanePosition& position,
        const api::IsoLaneVelocity& velocity) const override;

    api::LanePosition DoToLanePosition(
        const api::GeoPosition&) const override {
      // TODO(maddog@tri.global) Implement when someone needs this.
      DRAKE_ABORT();
    }

    const api::LaneId id_;
    const Segment* segment_{};

    api::RBounds lane_bounds_;
    api::RBounds driveable_bounds_;


    std::vector<Record> records_;
    double cycle_length_{};
  };


  class Segment : public api::Segment {
   public:
    Segment(const api::SegmentId& id,
            const Junction* junction, const Lane* lane)
        : id_(id), junction_(junction), lane_(lane) {}

    virtual ~Segment() {}

   private:
    const api::SegmentId do_id() const override { return id_; }

    const api::Junction* do_junction() const override { return junction_; }

    int do_num_lanes() const override { return 1; }

    const api::Lane* do_lane(int index) const override { return lane_; }

    api::SegmentId id_;
    const Junction* junction_{};
    const Lane* lane_{};
  };


  class Junction : public api::Junction {
   public:
    Junction(const api::JunctionId& id,
             const RoadGeometry* rg, const Segment* segment)
        : id_(id), road_geometry_(rg), segment_(segment) {}

    virtual ~Junction() {}

   private:
    const api::JunctionId do_id() const override { return id_; }

    const api::RoadGeometry* do_road_geometry() const override {
      return road_geometry_;
    }

    int do_num_segments() const override { return 1; }

    const api::Segment* do_segment(int index) const override {
      return segment_;
    }

    api::JunctionId id_;
    const RoadGeometry* road_geometry_{};
    const Segment* segment_{};
  };


  const api::RoadGeometryId do_id() const override { return id_; }

  int do_num_junctions() const override { return 1; }

  const api::Junction* do_junction(int index) const override {
    return &junction_;
  }

  int do_num_branch_points() const override { return 0; }

  const api::BranchPoint* do_branch_point(int index) const override {
    DRAKE_ABORT();
  }

  api::RoadPosition DoToRoadPosition(
      const api::GeoPosition& geo_pos,
      const api::RoadPosition& hint) const override {
    // TODO(maddog@tri.global) Implement when someone needs this.
    DRAKE_ABORT();
  }

  double do_linear_tolerance() const override {
    return source_->linear_tolerance();
  }

  double do_angular_tolerance() const override {
    return source_->angular_tolerance();
  }

  const api::RoadGeometryId id_;
  const api::RoadGeometry* const source_{};
  const Junction junction_;
  const Segment segment_;
  const Lane lane_;
};

}  // namespace utility
}  // namespace maliput
}  // namespace drake
