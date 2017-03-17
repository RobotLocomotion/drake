#include "drake/automotive/maliput/api/road_geometry.h"

#include <cmath>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "drake/automotive/maliput/api/branch_point.h"
#include "drake/automotive/maliput/api/junction.h"
#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/segment.h"
#include "drake/common/drake_assert.h"

namespace drake {
namespace maliput {
namespace api {

namespace {

GeoPosition LaneEndGeoPosition(const LaneEnd& lane_end) {
  return lane_end.lane->ToGeoPosition(
        LanePosition(
            (lane_end.end == LaneEnd::kStart) ? 0. : lane_end.lane->length(),
            0., 0.));
}


// Given a rotation representing the orientation of a (s,r,h) frame, return
// the rotation corresponding to "going the other way instead", i.e., the
// orientation of (-s,-r,h).  This is equivalent to a pre-rotation by PI in
// the s/r plane.
Rotation ReverseOrientation(const Rotation& rot) {
  const double ca = std::cos(rot.roll);
  const double sa = std::sin(rot.roll);
  const double cb = std::cos(rot.pitch);
  const double sb = std::sin(rot.pitch);
  const double cg = std::cos(rot.yaw);
  const double sg = std::sin(rot.yaw);
  return Rotation(std::atan2(-sa, ca),  // roll
                  std::atan2(-sb, cb),  // pitch
                  std::atan2(-sg, -cg));  // yaw
}


// Return the s/r/h orientation of the specified LaneEnd when travelling
// *out from* the lane at that end.
Rotation OrientationOutFromLane(const LaneEnd& lane_end) {
  switch (lane_end.end) {
    case LaneEnd::kStart: {
      return ReverseOrientation(lane_end.lane->GetOrientation({0., 0., 0.}));
    }
    case LaneEnd::kFinish: {
      return lane_end.lane->GetOrientation({lane_end.lane->length(), 0., 0.});
    }
    default: { DRAKE_ABORT(); }
  }
}


// Return the Cartesian distance between two GeoPositions.
double Distance(const GeoPosition& a, const GeoPosition& b) {
  const GeoPosition d {a.x - b.x, a.y - b.y, a.z - b.z};
  return std::sqrt((d.x * d.x) + (d.y * d.y) + (d.z * d.z));
}


// Apply a Rotation to a 3-vector (generically represented by a GeoPosition).
// TODO(maddog@tri.global)  This should probably be a method of Rotation, and or
//                          consolidated with something else somehow.
GeoPosition Rotate(const Rotation& rot, const GeoPosition& in) {
  const double sa = std::sin(rot.roll);
  const double ca = std::cos(rot.roll);
  const double sb = std::sin(rot.pitch);
  const double cb = std::cos(rot.pitch);
  const double sg = std::sin(rot.yaw);
  const double cg = std::cos(rot.yaw);

  return GeoPosition(
      ((cb * cg) * in.x) +
      ((-ca*sg + sa*sb*cg) * in.y) +
      ((sa*sg + ca*sb*cg) * in.z),

      ((cb*sg) * in.x) +
      ((ca*cg + sa*sb*sg) * in.y) +
      ((-sa*cg + ca*sb*sg) * in.z),

      ((-sb) * in.x) +
      ((sa*cb) * in.y) +
      ((ca*cb) * in.z));
}


// Return a distance measure (in radians) for two rotations that reflects
// the difference in frame orientations represented by the rotations.
double Distance(const Rotation& a, const Rotation& b) {
  // Compute transformed unit vectors of a frame.
  GeoPosition as = Rotate(a, {1., 0., 0.});
  GeoPosition ar = Rotate(a, {0., 1., 0.});
  GeoPosition ah = Rotate(a, {0., 0., 1.});
  // Compute transformed unit vectors of b frame.
  GeoPosition bs = Rotate(b, {1., 0., 0.});
  GeoPosition br = Rotate(b, {0., 1., 0.});
  GeoPosition bh = Rotate(b, {0., 0., 1.});
  // Compute angles between pairs of unit vectors.
  double ds = std::acos((as.x * bs.x) + (as.y * bs.y) + (as.z * bs.z));
  double dr = std::acos((ar.x * br.x) + (ar.y * br.y) + (ar.z * br.z));
  double dh = std::acos((ah.x * bh.x) + (ah.y * bh.y) + (ah.z * bh.z));

  return std::sqrt((ds * ds) + (dr * dr) + (dh * dh));
}

}  // namespace


std::vector<std::string> RoadGeometry::CheckInvariants() const {
  std::vector<std::string> failures;

  // Verify correctness of back-pointers/indexing in object hierarchy.
  for (int bpi = 0; bpi < num_branch_points(); ++bpi) {
    const BranchPoint* bp = branch_point(bpi);
    if (bp->road_geometry() != this) {
      std::stringstream ss;
      ss << "BranchPoint " << bp->id().id << " is owned by "
         << this->id().id << " (" << this
         << ") but claims to be owned by "
         << bp->road_geometry()->id().id << " ("
         << bp->road_geometry() << ").";
      failures.push_back(ss.str());
    }
  }
  for (int ji = 0; ji < num_junctions(); ++ji) {
    const Junction* jnx = junction(ji);
    if (jnx->road_geometry() != this) {
      std::stringstream ss;
      ss << "Junction " << jnx->id().id << " is owned by "
         << this->id().id << " (" << this << ") but claims to be owned by "
         << jnx->road_geometry()->id().id << " ("
         << jnx->road_geometry() << ").";
      failures.push_back(ss.str());
    }
    for (int si = 0; si < jnx->num_segments(); ++si) {
      const Segment* seg = jnx->segment(si);
      if (seg->junction() != jnx) {
        std::stringstream ss;
        ss << "Segment " << seg->id().id << " is owned by "
           << jnx->id().id << " (" << jnx << ") but claims to be owned by "
           << seg->junction()->id().id << " (" << seg->junction() << ").";
        failures.push_back(ss.str());
      }
      for (int li = 0; li < seg->num_lanes(); ++li) {
        const Lane* lane = seg->lane(li);
        if (lane->segment() != seg) {
          std::stringstream ss;
          ss << "Lane " << lane->id().id << " is owned by "
             << seg->id().id << " (" << seg << ") but claims to be owned by "
             << lane->segment()->id().id << " (" << lane->segment() << ").";
          failures.push_back(ss.str());
        }
        // Currently, only Lane has an index() accessor, because its the only
        // component for which the index is meaningful (e.g., adjacency of
        // lanes).
        if (lane->index() != li) {
          std::stringstream ss;
          ss << "Lane " << lane->id().id << " has index " << li
             << " but claims to have index " << lane->index() << ".";
          failures.push_back(ss.str());
        }
      }
    }
  }

  // Verify C1 continuity at branch-points (within declared tolerances).
  for (int bpi = 0; bpi < num_branch_points(); ++bpi) {
    const BranchPoint* bp = branch_point(bpi);
    // For each BranchPoint:
    //  - all branches should map to same GEO-space (x,y,z);
    //  - orientation *into* BranchPoint should be the same for all A-side
    //     branches;
    //  - orientation *into* BranchPoint should be the same for all B-side
    //     branches;
    //  - orientation *into* BranchPoint for A-side should be same as
    //     orientation *out of* BranchPoint for B-side.
    if ((bp->GetASide()->size() == 0) &&
        (bp->GetBSide()->size() == 0)) {
      std::stringstream ss;
      ss << "BranchPoint " << bp->id().id << " is empty.";
      failures.push_back(ss.str());
      continue;
    }

    const LaneEnd ref_end =
        (bp->GetASide()->size() > 0)
        ? bp->GetASide()->get(0)
        : bp->GetBSide()->get(0);
    // ...test GEO-space position similarity.
    const GeoPosition ref_geo = LaneEndGeoPosition(ref_end);
    const auto test_geo_position = [&](const LaneEndSet& ends) {
      for (int bi = 0; bi < ends.size(); ++bi) {
        const LaneEnd le = ends.get(bi);
        const double d = Distance(ref_geo, LaneEndGeoPosition(le));
        if (d > linear_tolerance()) {
          std::stringstream ss;
          ss << "Lane " << le.lane->id().id
             << ((le.end == LaneEnd::kStart) ? "[start]" : "[end]")
             << " position is off by " << d << ".";
          failures.push_back(ss.str());
        }
      }
    };
    test_geo_position(*(bp->GetASide()));
    test_geo_position(*(bp->GetBSide()));
    // ...test orientation similarity.
    const Rotation ref_rot =
        (bp->GetASide()->size() > 0)
        ? OrientationOutFromLane(bp->GetASide()->get(0))
        : ReverseOrientation(OrientationOutFromLane(bp->GetBSide()->get(0)));
    const auto test_orientation = [&](const LaneEndSet& ends,
                                      const Rotation& reference) {
      for (int bi = 0; bi < ends.size(); ++bi) {
        const LaneEnd le = ends.get(bi);
        const double d = Distance(reference, OrientationOutFromLane(le));
        if (d > angular_tolerance()) {
          std::stringstream ss;
          ss << "Lane " << le.lane->id().id
             << ((le.end == LaneEnd::kStart) ? "[start]" : "[end]")
             << " orientation is off by " << d << ".";
          failures.push_back(ss.str());
        }
      }
    };
    test_orientation(*(bp->GetASide()), ref_rot);
    test_orientation(*(bp->GetBSide()), ReverseOrientation(ref_rot));
  }

  // Check that Lane left/right relationships within a Segment are
  // geometrically sound.
  // TODO(maddog@tri.global)  Implement this.

  return failures;
}

}  // namespace api
}  // namespace maliput
}  // namespace drake
