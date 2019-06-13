#pragma once

#include <unordered_set>
#include <vector>

#include "drake/automotive/deprecated.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/api/segment.h"

namespace drake {
namespace maliput {
namespace utility {

/// Finds all Segments connected to @p seed_segment via confluent Lanes.
///
/// This function performs a breadth-first search over the graph of Segments,
/// Lanes, and BranchPoints originating at `seed_segment`.  It does not
/// explore any other elements and does not require these elements to have
/// valid ownership pointers upwards to Junctions or to a RoadGeometry.
///
/// @returns an unordered_set of Segments connected to `seed_segment`,
///          including `seed_segment` itself.
std::unordered_set<const api::Segment*>
FindConfluentSegments(const api::Segment* seed_segment);


/// Analyzes how Segments in @p road_geometry are connected via confluency
/// of their Lanes at BranchPoints.
///
/// Two Lanes which are confluent at a BranchPoint necessarily overlap near
/// the BranchPoint, so the Segments which own those Lanes ought to belong
/// to a common Junction. The output of this function is thus a lower-bound
/// for how Segments should be grouped together into Junctions, which can
/// be used for verifying or synthesizing (approximately) the Junction
/// structure.  (This function will not detect Lanes which have overlapping
/// geometries independent of their BranchPoints.)
///
/// @returns the set of
/// <a href="https://en.wikipedia.org/wiki/Connected_component_(graph_theory)">
///          connected components</a> of Segments, as a vector of
///          unordered_sets.  The ordering of the components in the vector is
///          arbitrary.  Each Segment in @p road_geometry shall belong
///          to exactly one component.
std::vector<std::unordered_set<const api::Segment*>>
AnalyzeConfluentSegments(const api::RoadGeometry* road_geometry);


}  // namespace utility
}  // namespace maliput
}  // namespace drake
