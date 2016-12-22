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

/// A toy adapter class that makes an api::RoadGeometry look like it has
/// a single inifinitely long Lane.  Its primary (and perhaps only) purpose
/// is to facilitate demos until proper hybrid system support is available
/// in drake.
///
/// Caveats:
///  * Only works with a RoadGeometry that has one-lane-per-segment.
///  * If no path for the circuit is specified, then the source
///    api::RoadGeometry must have no dead-ends.
class InfiniteCircuitRoad : public api::RoadGeometry {
 public:
  /// An element of the circuitous path traversed by an InfiniteCircuitRoad.
  struct Record {
    /// A pointer to an api::Lane in the wrapped api::RoadGeometry.
    const api::Lane* lane{};
    /// Where along the path-length of the circuit this element begins.
    double start_circuit_s{};
    // TODO(maddog)  Ditch this (always == start + lane->length()).
    /// Where along the path-length of the circuit this element ends.
    double end_circuit_s{};
    /// True if the lane should be traversed in the reverse direction, i.e.,
    /// starting at s = lane->length() instead of s = 0.
    bool is_reversed{};
  };


  /// InfiniteCircuitRoad's implementation of api::Junction.
  class Junction : public api::Junction {
   public:
    Junction(const api::JunctionId& id, const InfiniteCircuitRoad* road);
    virtual ~Junction() {}

   private:
    const api::JunctionId do_id() const override;
    const api::RoadGeometry* do_road_geometry() const override;
    int do_num_segments() const override;
    const api::Segment* do_segment(int index) const override;

    const api::JunctionId id_;
    const InfiniteCircuitRoad* const road_{};
  };


  /// InfiniteCircuitRoad's implementation of api::Segment.
  class Segment : public api::Segment {
   public:
    Segment(const api::SegmentId& id, const InfiniteCircuitRoad* road);
    virtual ~Segment() {}

   private:
    const api::SegmentId do_id() const override;
    const api::Junction* do_junction() const override;
    int do_num_lanes() const override;
    const api::Lane* do_lane(int index) const;

    api::SegmentId id_;
    const InfiniteCircuitRoad* const road_{};
  };


  /// InfiniteCircuitRoad's implementation of api::Lane.
  class Lane : public api::Lane {
   public:
    Lane(const api::LaneId& id, const InfiniteCircuitRoad* road,
         const api::RoadGeometry* source,
         const api::LaneEnd& start,
         const std::vector<const api::Lane*>& path);

    virtual ~Lane() {}

    /// Returns the actual length of a single cycle (despite the illusion that
    /// the road is infinitely long).
    double cycle_length() const { return cycle_length_; }

    /// Returns the position within the fixed-length circuit of the given
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

    /// Projects the given LanePosition @param lane_pos on the "infinite Lane"
    /// back to a RoadPosition in the source RoadGeometry.
    ///
    /// @returns a pair of:
    /// - RoadPosition indicating the source Lane and position on that Lane;
    /// - bool indicating travel should be reversed in the source Lane with
    ///   respect to +s travel in the Infinite Lane --- e.g., if true, then
    ///   +s motion in the Infinite Lane corresponds to -s motion in the
    ///   source Lane.
    std::pair<api::RoadPosition, bool> ProjectToSourceRoad(
        const api::LanePosition& lane_pos) const;

   private:
    const api::LaneId do_id() const override;
    const api::Segment* do_segment() const override;
    int do_index() const override;
    const api::Lane* do_to_left() const override;
    const api::Lane* do_to_right() const override;
    const api::BranchPoint* DoGetBranchPoint(
        const api::LaneEnd::Which which_end) const override;
    const api::LaneEndSet* DoGetConfluentBranches(
        const api::LaneEnd::Which which_end) const override;
    const api::LaneEndSet* DoGetOngoingBranches(
        const api::LaneEnd::Which which_end) const override;
    std::unique_ptr<api::LaneEnd> DoGetDefaultBranch(
        const api::LaneEnd::Which which_end) const override;
    double do_length() const override;
    api::RBounds do_lane_bounds(double) const override;
    api::RBounds do_driveable_bounds(double) const override;
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
    const InfiniteCircuitRoad* road_{};
    api::RBounds lane_bounds_;
    api::RBounds driveable_bounds_;
    std::vector<Record> records_;
    double cycle_length_{};
  };



  /// Constructs an InfiniteCircuitRoad wrapping @p source, using
  /// @p start as the starting point in the search for a closed circuit.
  ///
  /// NB:  All the real construction work happens in the constructor for
  /// InfiniteCircuitRoad::Lane.
  InfiniteCircuitRoad(const api::RoadGeometryId& id,
                      const api::RoadGeometry* source,
                      const api::LaneEnd& start,
                      const std::vector<const api::Lane*>& path);

  virtual ~InfiniteCircuitRoad();

  /// Returns the sole Lane component emulated by this api::RoadGeometry.
  const Lane* lane() const { return &lane_; }

  /// Returns a pointer to the underlying source RoadGeometry.
  const api::RoadGeometry* source() const { return source_; }

 private:
  const api::RoadGeometryId do_id() const override;
  int do_num_junctions() const override;
  const api::Junction* do_junction(int index) const override;
  int do_num_branch_points() const override;
  const api::BranchPoint* do_branch_point(int index) const override;
  api::RoadPosition DoToRoadPosition(
      const api::GeoPosition& geo_pos,
      const api::RoadPosition& hint) const override;
  double do_linear_tolerance() const override;
  double do_angular_tolerance() const override;

  const api::RoadGeometryId id_;
  const api::RoadGeometry* const source_{};
  const Junction junction_;
  const Segment segment_;
  const Lane lane_;
};

}  // namespace utility
}  // namespace maliput
}  // namespace drake
