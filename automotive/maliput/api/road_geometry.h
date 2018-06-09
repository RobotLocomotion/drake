#pragma once

#include <string>
#include <vector>

#include "drake/automotive/maliput/api/branch_point.h"
#include "drake/automotive/maliput/api/junction.h"
#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/automotive/maliput/api/segment.h"
#include "drake/automotive/maliput/api/type_specific_identifier.h"
#include "drake/common/drake_copyable.h"

namespace drake {
namespace maliput {
namespace api {

class BranchPoint;
class Junction;


/// Persistent identifier for a RoadGeometry element.
using RoadGeometryId = TypeSpecificIdentifier<class RoadGeometry>;


/// Abstract API for the geometry of a road network, including both
/// the network topology and the geometry of its embedding in 3-space.
// TODO(maddog@tri.global)  This entire API should be templated on a
//                          scalar type T like everything else in drake.
class RoadGeometry {
 public:
  class IdIndex;

  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RoadGeometry);

  virtual ~RoadGeometry() = default;

  /// Returns the persistent identifier.
  ///
  // TODO(maddog@tri.global)  Tie id into a tiling mechanism?
  const RoadGeometryId id() const { return do_id(); }

  // TODO(maddog@tri.global) Proper iterators over junctions and branch-points?

  /// Returns the number of Junctions in the RoadGeometry.
  ///
  /// Return value is non-negative.
  int num_junctions() const { return do_num_junctions(); }

  /// Returns the Junction indexed by @p index.
  ///
  /// @pre @p index must be >= 0 and < num_junctions().
  const Junction* junction(int index) const { return do_junction(index); }

  /// Returns the number of BranchPoints in the RoadGeometry.
  ///
  /// Return value is non-negative.
  int num_branch_points() const { return do_num_branch_points(); }

  /// Returns the BranchPoint indexed by @p index.
  ///
  /// @pre @p index must be >= 0 and < num_branch_points().
  const BranchPoint* branch_point(int index) const {
    return do_branch_point(index);
  }

  /// Accesses the IdIndex interface, which allows getting elements of
  /// the RoadGeometry's object graph by their unique id's.
  const IdIndex& ById() const { return DoById(); }

  /// Determines the RoadPosition corresponding to GeoPosition @p geo_position.
  ///
  /// If @p hint is non-null, its value is used to help determine the result.
  ///
  /// The return value is the RoadPosition of the point in the RoadGeometry's
  /// manifold which is, in the world frame, closest to @p geo_position.  If
  /// @p nearest_position is non-null, then it will be populated with the
  /// GeoPosition of that nearest point.  If @p distance is non-null, then it
  /// will be populated with the Cartesian distance from @p geo_position to
  /// that nearest point.
  ///
  /// This method guarantees that its result satisfies the condition that
  /// `result.lane->ToGeoPosition(result.pos)` is within `linear_tolerance()`
  /// of `*nearest_position`.
  ///
  /// The map from RoadGeometry to the world frame is not onto (as a bounded
  /// RoadGeometry cannot completely cover the unbounded Cartesian universe).
  /// If @p geo_position does represent a point contained within the volume
  /// of the RoadGeometry, then result @p distance is guaranteed to be less
  /// than or equal to `linear_tolerance()`.
  ///
  /// The map from RoadGeometry to world frame is not necessarily one-to-one.
  /// Different `(s,r,h)` coordinates from different Lanes, potentially from
  /// different Segments, may map to the same `(x,y,z)` world frame location.
  ///
  /// If @p geo_position is contained within the volumes of multiple Segments,
  /// then ToRoadPosition() will choose a Segment which yields the minimum
  /// height `h` value in the result.  If the chosen Segment has multiple
  /// Lanes, then ToRoadPosition() will choose a Lane which contains
  /// @p geo_position within its `lane_bounds()` if possible, and if that is
  /// still ambiguous, it will further select a Lane which minimizes the
  /// absolute value of the lateral `r` coordinate in the result.
  // TODO(maddog@tri.global)  Establish what effect `hint` has on the outcome.
  //                          Two notions:
  //                          1)  the hint helps to bootstrap a search;
  //                          2)  the hint helps to choose between multiple
  //                              equally valid solutions.
  //                          The general idea:  if one knows the RoadPosition
  //                          of an entity some small dT in the past, then one
  //                          might expect an updated RoadPosition which is
  //                          nearby (e.g., on the same Lane).
  RoadPosition ToRoadPosition(const GeoPosition& geo_position,
                              const RoadPosition* hint,
                              GeoPosition* nearest_position,
                              double* distance) const {
    return DoToRoadPosition(geo_position, hint, nearest_position, distance);
  }

  /// Returns the tolerance guaranteed for linear measurements (positions).
  double linear_tolerance() const {
    return do_linear_tolerance();
  }

  /// Returns the tolerance guaranteed for angular measurements (orientations).
  double angular_tolerance() const {
    return do_angular_tolerance();
  }

  /// Verifies certain invariants guaranteed by the API.
  ///
  /// Returns a vector of strings describing violations of invariants.
  /// Return value with size() == 0 indicates success.
  std::vector<std::string> CheckInvariants() const;

 protected:
  RoadGeometry() = default;

 private:
  /// @name NVI implementations of the public methods.
  /// These must satisfy the constraints/invariants of the
  /// corresponding public methods.
  ///@{
  virtual const RoadGeometryId do_id() const = 0;

  virtual int do_num_junctions() const = 0;

  virtual const Junction* do_junction(int index) const = 0;

  virtual int do_num_branch_points() const = 0;

  virtual const BranchPoint* do_branch_point(int index) const = 0;

  virtual const IdIndex& DoById() const = 0;

  virtual RoadPosition DoToRoadPosition(const GeoPosition& geo_pos,
                                        const RoadPosition* hint,
                                        GeoPosition* nearest_position,
                                        double* distance) const = 0;

  virtual double do_linear_tolerance() const = 0;

  virtual double do_angular_tolerance() const = 0;
  ///@}
};


/// Abstract interface for a collection of methods which allow accessing
/// objects in a RoadGeometry's object graph (Lanes, Segments, Junctions,
/// BranchPoints) by their unique id's.
class RoadGeometry::IdIndex {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(IdIndex);
  virtual ~IdIndex() = default;

  /// Returns the Lane identified by @p id, or `nullptr` if @p id is unknown.
  const Lane* GetLane(const LaneId& id) const { return DoGetLane(id); }

  /// Returns the Segment identified by @p id, or `nullptr` if @p id is
  /// unknown.
  const Segment* GetSegment(const SegmentId& id) const {
    return DoGetSegment(id);
  }

  /// Returns the Junction identified by @p id, or `nullptr` if @p id is
  /// unknown.
  const Junction* GetJunction(const JunctionId& id) const {
    return DoGetJunction(id);
  }

  /// Returns the BranchPoint identified by @p id, or `nullptr` if @p id is
  /// unknown.
  const BranchPoint* GetBranchPoint(const BranchPointId& id) const {
    return DoGetBranchPoint(id);
  }

 protected:
  IdIndex() = default;

 private:
  virtual const Lane* DoGetLane(const LaneId& id) const = 0;
  virtual const Segment* DoGetSegment(const SegmentId& id) const = 0;
  virtual const Junction* DoGetJunction(const JunctionId& id) const = 0;
  virtual const BranchPoint* DoGetBranchPoint(
      const BranchPointId& id) const = 0;
};


}  // namespace api
}  // namespace maliput
}  // namespace drake
