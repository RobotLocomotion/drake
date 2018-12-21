#include "drake/automotive/maliput/utility/generate_obj.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <functional>
#include <initializer_list>
#include <tuple>
#include <unordered_map>
#include <vector>

#include "fmt/ostream.h"

#include "drake/automotive/maliput/api/branch_point.h"
#include "drake/automotive/maliput/api/junction.h"
#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/api/segment.h"
#include "drake/automotive/maliput/utility/mesh.h"
#include "drake/automotive/maliput/utility/mesh_simplification.h"
#include "drake/common/drake_assert.h"

namespace drake {
namespace maliput {
namespace utility {

using mesh::GeoMesh;
using mesh::SrhFace;
using mesh::SimplifyMeshFaces;

namespace {

// Traverses @p lane, generating a cover of the surface with with quads
// (4-vertex faces) which are added to @p mesh.  The quads are squares in
// the (s,r) space of the lane.
//
// @param mesh  the GeoMesh which will receive the quads
// @param lane  the api::Lane to cover with quads
// @param grid_unit  size of each quad (length of edge in s and r dimensions)
// @param use_driveable_bounds  if true, use the lane's driveable_bounds()
//        to determine the lateral extent of the coverage; otherwise, use
//        lane_bounds()
// @param elevation a function taking `(s, r)` as parameters and returning
//        the corresponding elevation `h`, to yield a quad vertex `(s, r, h)`
void CoverLaneWithQuads(
    GeoMesh* mesh, const api::Lane* lane,
    double grid_unit, bool use_driveable_bounds,
    const std::function<double(double, double)>& elevation) {
  const double linear_tolerance =
    lane->segment()->junction()->road_geometry()->linear_tolerance();
  const double s_max = lane->length();
  for (double s0 = 0, s1; s0 < s_max; s0 = s1) {
    s1 = s0 + grid_unit;
    if (s1 > s_max - linear_tolerance) { s1 = s_max; }

    const api::RBounds rb0 = use_driveable_bounds ?
        lane->driveable_bounds(s0) : lane->lane_bounds(s0);
    const api::RBounds rb1 = use_driveable_bounds ?
        lane->driveable_bounds(s1) : lane->lane_bounds(s1);

    // Left side of lane (r >= 0).
    {
      double r00 = 0.;
      double r10 = 0.;
      while ((r00 < rb0.max()) && (r10 < rb1.max())) {
        const double r01 = std::min(r00 + grid_unit, rb0.max());
        const double r11 = std::min(r10 + grid_unit, rb1.max());
        //
        // (s1,r11) o <-- o (s1,r10)       ^ +s
        //          |     ^                |
        //          v     |          +r <--o
        // (s0,r01) o --> * (s0,r00)
        //
        SrhFace srh_face({
            {s0, r00, elevation(s0, r00)},
            {s1, r10, elevation(s1, r10)},
            {s1, r11, elevation(s1, r11)},
            {s0, r01, elevation(s0, r01)}}, {0., 0., 1.});
        mesh->PushFace(srh_face.ToGeoFace(lane));

        r00 += grid_unit;
        r10 += grid_unit;
      }
    }
    // Right side of lane (r <= 0).
    {
      double r00 = 0.;
      double r10 = 0.;
      while ((r00 > rb0.min()) && (r10 > rb1.min())) {
        const double r01 = std::max(r00 - grid_unit, rb0.min());
        const double r11 = std::max(r10 - grid_unit, rb1.min());
        //
        // (s1,r10) o <-- o (s1,r11)  ^ +s
        //          |     ^           |
        //          v     |           o--> -r
        // (s0,r00) * --> o (s0,r01)
        //
        SrhFace srh_face({
            {s0, r00, elevation(s0, r00)},
            {s0, r01, elevation(s0, r01)},
            {s1, r11, elevation(s1, r11)},
            {s1, r10, elevation(s1, r10)}}, {0., 0., 1.});
        mesh->PushFace(srh_face.ToGeoFace(lane));

        r00 -= grid_unit;
        r10 -= grid_unit;
      }
    }
  }
}


// Adds faces to @p mesh which draw stripes along the lane_bounds() of
// @p lane.
//
// @param mesh  the GeoMesh which will receive the faces
// @param lane  the api::Lane to provide the bounds and surface
// @param grid_unit  longitudinal size (s dimension) of each face
// @param h_offset  h value of each vertex (height above road surface)
// @param stripe_width  width (r dimension) of each stripe
void StripeLaneBounds(GeoMesh* mesh, const api::Lane* lane,
                      double grid_unit, double h_offset,
                      double stripe_width) {
  const double half_stripe = 0.5 * stripe_width;
  const double linear_tolerance =
    lane->segment()->junction()->road_geometry()->linear_tolerance();
  const double s_max = lane->length();
  for (double s0 = 0, s1; s0 < s_max; s0 = s1) {
    s1 = s0 + grid_unit;
    if (s1 > s_max - linear_tolerance) { s1 = s_max; }

    api::RBounds rb0 = lane->lane_bounds(s0);
    api::RBounds rb1 = lane->lane_bounds(s1);

    // Left side of lane.
    {
      SrhFace srh_face({
          {s0, rb0.max() - half_stripe, h_offset},
          {s1, rb1.max() - half_stripe, h_offset},
          {s1, rb1.max() + half_stripe, h_offset},
          {s0, rb0.max() + half_stripe, h_offset}}, {0., 0., 1.});
      mesh->PushFace(srh_face.ToGeoFace(lane));
    }
    // Right side of lane.
    {
      SrhFace srh_face({
          {s0, rb0.min() - half_stripe, h_offset},
          {s1, rb1.min() - half_stripe, h_offset},
          {s1, rb1.min() + half_stripe, h_offset},
          {s0, rb0.min() + half_stripe, h_offset}}, {0., 0., 1.});
      mesh->PushFace(srh_face.ToGeoFace(lane));
    }
  }
}


// Adds faces to @p mesh which draw a simple triangular arrow in the
// `Lane`-frame of @p lane.  The width of the arrow is fixed at 80% of
// the lane_bounds() of @p lane at the base of the arrow.
//
// @param mesh  the GeoMesh which will receive the faces
// @param lane  the api::Lane to provide the bounds and surface
// @param grid_unit  size of each quad (length of edge in s and r dimensions)
// @param s_offset  longitudinal offset of the base of the arrow from the
//                  beginning (s = 0) of @p lane
// @param s_size  length of the arrow from base to tip
// @param h_offset  h value of each vertex (height above road surface)
void DrawLaneArrow(GeoMesh* mesh, const api::Lane* lane, double grid_unit,
                   double s_offset, double s_size, double h_offset) {
  DRAKE_DEMAND(s_offset >= 0.);
  DRAKE_DEMAND((s_offset + s_size) <= lane->length());
  const double kRelativeWidth = 0.8;

  const api::RBounds rb0 = lane->lane_bounds(s_offset);

  const int max_num_s_units = static_cast<int>(std::ceil(s_size / grid_unit));

  const double rl_size = rb0.max() * kRelativeWidth;
  const double rr_size = -rb0.min() * kRelativeWidth;
  const int max_num_rl_units = static_cast<int>(std::ceil(rl_size / grid_unit));
  const int max_num_rr_units = static_cast<int>(std::ceil(rr_size / grid_unit));

  const int num_units = std::max(max_num_s_units,
                                 std::max(max_num_rl_units,
                                          max_num_rr_units));
  DRAKE_DEMAND(num_units >= 1);
  const double s_unit = s_size / num_units;
  const double rl_unit = rl_size / num_units;
  const double rr_unit = rr_size / num_units;

  int num_r_units = num_units;
  for (int si = 0; si < num_units; ++si) {
    double s0 = s_offset + (si * s_unit);
    double s1 = s_offset + ((si + 1.) * s_unit);
    // Left side of lane.
    {
      double r00 = 0.;
      double r10 = 0.;
      for (int ri = 0; ri < (num_r_units - 1); ++ri) {
        const double r01 = r00 + rl_unit;
        const double r11 = r10 + rl_unit;
        //
        // (s1,r11) o <-- o (s1,r10)       ^ +s
        //          |     ^                |
        //          v     |          +r <--o
        // (s0,r01) o --> * (s0,r00)
        //
        SrhFace srh_face({
            {s0, r00, h_offset},
            {s1, r10, h_offset},
            {s1, r11, h_offset},
            {s0, r01, h_offset}}, {0., 0., 1.});
        mesh->PushFace(srh_face.ToGeoFace(lane));

        r00 += rl_unit;
        r10 += rl_unit;
      }
      //                o (s1,r10)       ^ +s
      //              / ^                |
      //            /   |          +r <--o
      // (s0,r01) o --> * (s0,r00)
      SrhFace srh_face({
          {s0, r00, h_offset},
          {s1, r10, h_offset},
          {s0, r00 + rl_unit, h_offset}}, {0., 0., 1.});
      mesh->PushFace(srh_face.ToGeoFace(lane));
    }
    // Right side of lane.
    {
      double r00 = 0.;
      double r10 = 0.;
      for (int ri = 0; ri < (num_r_units - 1); ++ri) {
        const double r01 = r00 - rr_unit;
        const double r11 = r10 - rr_unit;
        //
        // (s1,r10) o <-- o (s1,r11)  ^ +s
        //          |     ^           |
        //          v     |           o--> -r
        // (s0,r00) * --> o (s0,r01)
        //
        SrhFace srh_face({
            {s0, r00, h_offset},
            {s0, r01, h_offset},
            {s1, r11, h_offset},
            {s1, r10, h_offset}}, {0., 0., 1.});
        mesh->PushFace(srh_face.ToGeoFace(lane));

        r00 -= rr_unit;
        r10 -= rr_unit;
      }
      //
      // (s1,r10) o                 ^ +s
      //          | \               |
      //          v   \             o--> -r
      // (s0,r00) * --> o (s0,r01)
      //
      SrhFace srh_face({
          {s0, r00, h_offset},
          {s0, r00 - rr_unit, h_offset},
          {s1, r10, h_offset}}, {0., 0., 1.});
      mesh->PushFace(srh_face.ToGeoFace(lane));
    }

    num_r_units -= 1;
  }
}


// Marks the start and finish ends of @p lane with arrows, rendered into
// @p mesh.
//
// @param mesh  the GeoMesh which will receive the arrows
// @param lane  the api::Lane to provide the surface
// @param grid_unit  size of each quad (length of edge in s and r dimensions)
// @param h_offset  h value of each vertex (height above road surface)
void MarkLaneEnds(GeoMesh* mesh, const api::Lane* lane, double grid_unit,
                  double h_offset) {
  // To avoid crossing boundaries (and tripping assertions) due to
  // numeric precision issues, we will nudge the arrows inward from
  // the ends of the lanes by the RoadGeometry's linear_tolerance().
  const double nudge =
      lane->segment()->junction()->road_geometry()->linear_tolerance();
  const double max_length = 0.3 * lane->length();
  // Arrows are sized relative to their respective ends.
  const api::RBounds start_rb = lane->lane_bounds(0.);
  const double start_s_size = std::min(max_length,
                                       (start_rb.max() - start_rb.min()));

  const api::RBounds finish_rb = lane->lane_bounds(lane->length());
  const double finish_s_size = std::min(max_length,
                                        (finish_rb.max() - finish_rb.min()));

  DrawLaneArrow(mesh, lane, grid_unit,
                0. + nudge, start_s_size, h_offset);
  DrawLaneArrow(mesh, lane, grid_unit,
                lane->length() - finish_s_size - nudge, finish_s_size,
                h_offset);
}


// Calculates an appropriate grid-unit size for @p lane.
double PickGridUnit(const api::Lane* lane,
                    double max_size, double min_resolution) {
  double result = max_size;
  const api::RBounds rb0 = lane->lane_bounds(0.);
  const api::RBounds rb1 = lane->lane_bounds(lane->length());
  result = std::min(result, (rb0.max() - rb0.min()) / min_resolution);
  result = std::min(result, (rb1.max() - rb1.min()) / min_resolution);
  result = std::min(result, lane->length() / min_resolution);
  return result;
}


// Renders a BranchPoint @p branch_point as a collection of pointy
// arrows for each branch.  @p base_elevation is the desired elevation
// of the center of the rendering (above the road surface), and
// @p height is the vertical size of rendering.  The actual elevation
// may be raised in order to avoid overlapping other nearby
// BranchPoints.  @p mesh is the mesh into which the rendering occurs.
// @p previous_centers is a list of the world-frame positions of the
// centers of previously rendered BranchPoints (in order to avoid
// overlaps with them); this list will be updated with the rendered
// center of this BranchPoint.
void RenderBranchPoint(
    const api::BranchPoint* const branch_point,
    const double base_elevation, const double height,
    GeoMesh* mesh,
    std::vector<api::GeoPosition>* previous_centers) {
  if ((branch_point->GetASide()->size() == 0) &&
      (branch_point->GetBSide()->size() == 0)) {
    // No branches?  Odd, but, oh, well... nothing to do here.
    return;
  }

  // Arbitrarily pick one of the LaneEnds in the BranchPoint as a reference
  // for its geometry (e.g., *where* is the BranchPoint).
  const api::LaneEnd reference_end =
      (branch_point->GetASide()->size() > 0) ?
      branch_point->GetASide()->get(0) :
      branch_point->GetBSide()->get(0);
  const double reference_end_s =
      (reference_end.end == api::LaneEnd::kStart) ? 0. :
      reference_end.lane->length();
  const api::RBounds reference_bounds =
      reference_end.lane->lane_bounds(reference_end_s);
  const double sr_margin = reference_bounds.max() - reference_bounds.min();
  const double h_margin = height;

  // Choose an elevation that keeps this BranchPoint out of the way
  // of previously rendered BranchPoints.
  double elevation = base_elevation;
  bool has_conflict = true;
  while (has_conflict) {
    // Calculate center in world-frame with current elevation.
    const api::LanePosition center_srh(
        (reference_end.end == api::LaneEnd::kStart) ? 0. :
        reference_end.lane->length(),
        0., elevation);
    const api::Rotation orientation =
        reference_end.lane->GetOrientation(center_srh);
    const api::GeoPosition center_xyz =
        reference_end.lane->ToGeoPosition(center_srh);

    has_conflict = false;
    // Compare center against every already-rendered center....
    // If distance in sr-plane is too close and distance along h-axis is
    // too close, then increase elevation and try again.
    for (const api::GeoPosition& previous_xyz : *previous_centers) {
      const Vector3<double> delta_xyz = previous_xyz.xyz() - center_xyz.xyz();
      const Vector3<double> delta_srh =
          orientation.matrix().transpose() * delta_xyz;

      if ((Vector2<double>(delta_srh.x(), delta_srh.y()).norm() < sr_margin) &&
          (std::abs(delta_srh.z()) < h_margin)) {
        has_conflict = true;
        elevation += height;
        break;
      }
    }

    if (!has_conflict) {
      previous_centers->emplace_back(center_xyz);
    }
  }

  // Finally, draw the BranchPoint as:
  // - a single vertical diamond, facing into the lane of reference_end;
  // - for each branch (LaneEnd), an arrow formed from a pair of very
  //   pointy trapezoids (one in the sr-plane, one in the sh-plane) pointing
  //   into the lane.
  static const double kWidthFactor = 0.1;
  static const double kTipFactor = 0.1;
  static const double kLengthFactor = 1.0;
  static const double kMaxLengthFraction = 0.4;

  // Helper to draw a LaneEnd as either diamond or arrow.
  const auto draw_branch =
      [elevation, height, &mesh](const api::LaneEnd& lane_end,
                                 bool as_diamond) {
    const double end_s =
      (lane_end.end == api::LaneEnd::kStart) ? 0. : lane_end.lane->length();
    const api::RBounds r_bounds = lane_end.lane->lane_bounds(end_s);

    const double half_width =
      (r_bounds.max() - r_bounds.min()) * kWidthFactor * 0.5;
    const double length =
      std::min(kMaxLengthFraction * lane_end.lane->length(),
               kLengthFactor * (r_bounds.max() - r_bounds.min())) *
      ((lane_end.end == api::LaneEnd::kStart) ? 1. : -1);

    const double left_r =
      half_width * ((lane_end.end == api::LaneEnd::kStart) ? 1. : -1);
    const double right_r = -left_r;

    if (as_diamond) {
      SrhFace srh_face({
          {end_s, 0., elevation - (0.5 * height)},
          {end_s, right_r, elevation},
          {end_s, 0., elevation + (0.5 * height)},
          {end_s, left_r, elevation}},
        api::LanePosition{(end_s == 0. ? 1. : -1), 0., 0.});
      mesh->PushFace(srh_face.ToGeoFace(lane_end.lane));
    } else {
      SrhFace srh_face1({
          {end_s, left_r, elevation},
          {end_s, right_r, elevation},
          {end_s + length, right_r * kTipFactor, elevation},
          {end_s + length, left_r * kTipFactor, elevation}},
        api::LanePosition{0., 0., 1.});
      SrhFace srh_face2({
          {end_s, 0., elevation - (0.5 * height)},
          {end_s, 0., elevation + (0.5 * height)},
          {end_s + length, 0., elevation + (0.5 * kTipFactor * height)},
          {end_s + length, 0., elevation - (0.5 * kTipFactor * height)}
        },
        api::LanePosition{0., (length > 0. ? 1. : -1.), 0.});
      mesh->PushFace(srh_face1.ToGeoFace(lane_end.lane));
      mesh->PushFace(srh_face2.ToGeoFace(lane_end.lane));
    }
  };

  // Helper to draw all LaneEnds in a LaneEndSet as arrows.
  const auto draw_arrows = [&draw_branch](const api::LaneEndSet* set) {
    for (int i = 0; i < set->size(); ++i) {
      draw_branch(set->get(i), false);
    }
  };

  draw_branch(reference_end, true /* as_diamond */);
  draw_arrows(branch_point->GetASide());
  draw_arrows(branch_point->GetBSide());
}

GeoMesh SimplifyMesh(const GeoMesh& mesh, const ObjFeatures& features) {
  if (features.simplify_mesh_threshold == 0.) {
    return mesh;  // Passes given mesh unmodified.
  }
  return SimplifyMeshFaces(mesh, features.simplify_mesh_threshold);
}

void RenderSegment(const api::Segment* segment,
                   const ObjFeatures& features,
                   GeoMesh* asphalt_mesh,
                   GeoMesh* lane_mesh,
                   GeoMesh* marker_mesh,
                   GeoMesh* h_bounds_mesh) {
  const double base_grid_unit = PickGridUnit(
      segment->lane(0), features.max_grid_unit,
      features.min_grid_resolution);
  {
    // Lane 0 should be as good as any other for driveable-bounds.
    GeoMesh driveable_mesh;
    CoverLaneWithQuads(&driveable_mesh, segment->lane(0),
                       base_grid_unit,
                       true /*use_driveable_bounds*/,
                       [](double, double) { return 0.; });
    asphalt_mesh->AddFacesFrom(SimplifyMesh(driveable_mesh, features));
  }

  if (features.draw_elevation_bounds) {
    GeoMesh upper_h_bounds_mesh, lower_h_bounds_mesh;
    CoverLaneWithQuads(
        &upper_h_bounds_mesh,
        segment->lane(0),
        base_grid_unit,
        true /*use_driveable_bounds*/,
        [&segment](double s, double r) {
          return segment->lane(0)->elevation_bounds(s, r).max(); });
    CoverLaneWithQuads(
        &lower_h_bounds_mesh,
        segment->lane(0),
        base_grid_unit,
        true /*use_driveable_bounds*/,
        [&segment](double s, double r) {
          return segment->lane(0)->elevation_bounds(s, r).min(); });
    h_bounds_mesh->AddFacesFrom(SimplifyMesh(upper_h_bounds_mesh, features));
    h_bounds_mesh->AddFacesFrom(SimplifyMesh(lower_h_bounds_mesh, features));
  }
  for (int li = 0; li < segment->num_lanes(); ++li) {
    const api::Lane* lane = segment->lane(li);
    const double grid_unit = PickGridUnit(lane,
                                          features.max_grid_unit,
                                          features.min_grid_resolution);
    if (features.draw_lane_haze) {
      GeoMesh haze_mesh;
      CoverLaneWithQuads(&haze_mesh, lane, grid_unit,
                         false /*use_driveable_bounds*/,
                         [&features](double, double) {
                           return features.lane_haze_elevation;
                         });
      lane_mesh->AddFacesFrom(SimplifyMesh(haze_mesh, features));
    }
    if (features.draw_stripes) {
      GeoMesh stripes_mesh;
      StripeLaneBounds(&stripes_mesh, lane, grid_unit,
                       features.stripe_elevation,
                       features.stripe_width);
      marker_mesh->AddFacesFrom(SimplifyMesh(stripes_mesh, features));
    }
    if (features.draw_arrows) {
      GeoMesh arrows_mesh;
      MarkLaneEnds(&arrows_mesh, lane, grid_unit,
                   features.arrow_elevation);
      marker_mesh->AddFacesFrom(SimplifyMesh(arrows_mesh, features));
    }
  }
}


bool IsSegmentRenderedNormally(const api::SegmentId& id,
                               const std::vector<api::SegmentId>& highlights) {
  if (highlights.empty()) {
    return true;
  }
  for (const api::SegmentId& highlighted_id : highlights) {
    if (id == highlighted_id) {
      return true;
    }
  }
  return false;
}

}  // namespace


void GenerateObjFile(const api::RoadGeometry* rg,
                     const std::string& dirpath,
                     const std::string& fileroot,
                     const ObjFeatures& features) {
  GeoMesh asphalt_mesh;
  GeoMesh lane_mesh;
  GeoMesh marker_mesh;
  GeoMesh h_bounds_mesh;
  GeoMesh branch_point_mesh;

  GeoMesh grayed_asphalt_mesh;
  GeoMesh grayed_lane_mesh;
  GeoMesh grayed_marker_mesh;

  // Walk the network.
  for (int ji = 0; ji < rg->num_junctions(); ++ji) {
    const api::Junction* junction = rg->junction(ji);
    for (int si = 0; si < junction->num_segments(); ++si) {
      const api::Segment* segment = junction->segment(si);
      // TODO(maddog@tri.global)  Id's need well-defined comparison semantics.
      if (IsSegmentRenderedNormally(segment->id(),
                                    features.highlighted_segments)) {
        RenderSegment(segment, features,
                      &asphalt_mesh, &lane_mesh, &marker_mesh, &h_bounds_mesh);
      } else {
        RenderSegment(segment, features,
                      &grayed_asphalt_mesh, &grayed_lane_mesh,
                      &grayed_marker_mesh, &h_bounds_mesh);
      }
    }
  }

  if (features.draw_branch_points) {
    std::vector<api::GeoPosition> rendered_centers;
    for (int bpi = 0; bpi < rg->num_branch_points(); ++bpi) {
      const api::BranchPoint* branch_point = rg->branch_point(bpi);
      RenderBranchPoint(branch_point,
                        features.branch_point_elevation,
                        features.branch_point_height,
                        &branch_point_mesh,
                        &rendered_centers);
    }
  }

  const std::string kLaneHaze("lane_haze");
  const std::string kMarkerPaint("marker_paint");
  const std::string kBlandAsphalt("bland_asphalt");
  const std::string kBranchPointGlow("branch_point_glow");
  const std::string kHBoundsHaze("h_bounds_haze");

  const std::string kGrayedLaneHaze("grayed_lane_haze");
  const std::string kGrayedMarkerPaint("grayed_marker_paint");
  const std::string kGrayedBlandAsphalt("grayed_bland_asphalt");

  const std::string obj_filename = fileroot + ".obj";
  const std::string mtl_filename = fileroot + ".mtl";

  // Create the requested OBJ file.
  {
    // Figure out the fixed-point precision necessary to render OBJ vertices
    // with enough precision relative to linear_tolerance().
    //
    // Given linear_tolerance ε, we conservatively want to bound the rendering
    // error per component to `ε / (sqrt(3) * 10)`.  The `sqrt(3)` is
    // because the worst-case error in total 3-space distance is `sqrt(3)`
    // times the per-component error.  The `10` is a fudge-factor to ensure
    // that the "rendering error in an OBJ vertex with respect to the
    // maliput-expressed value" is within 10% of the "error-bound between
    // the maliput-expressed position and the underlying ground-truth".
    // In other words, we're aiming for the rendered vertex to be within
    // 110% ε of the ground-truth position.
    //
    // The bound on error due to rounding to `n` places is `0.5 * 10^(-n)`,
    // so we want `n` such that `0.5 * 10^(-n) < ε / (sqrt(3) * 10)`.
    // This yields:  `n > log10(sqrt(3) * 5) - log10(ε)`.
    DRAKE_DEMAND(rg->linear_tolerance() > 0.);
    const int precision =
        std::max(0., std::ceil(std::log10(std::sqrt(3.) * 5.) -
                               std::log10(rg->linear_tolerance())));

    std::ofstream os(dirpath + "/" + obj_filename, std::ios::binary);
    fmt::print(os,
               R"X(# GENERATED BY maliput::utility::GenerateObjFile()
#
# DON'T BE A HERO.  Do not edit by hand.

mtllib {}
)X",
               mtl_filename);
    int vertex_index_offset = 0;
    int normal_index_offset = 0;
    std::tie(vertex_index_offset, normal_index_offset) =
        asphalt_mesh.EmitObj(os, kBlandAsphalt,
                             precision, features.origin,
                             vertex_index_offset, normal_index_offset);
    std::tie(vertex_index_offset, normal_index_offset) =
        lane_mesh.EmitObj(os, kLaneHaze,
                          precision, features.origin,
                          vertex_index_offset, normal_index_offset);
    std::tie(vertex_index_offset, normal_index_offset) =
        marker_mesh.EmitObj(os, kMarkerPaint,
                            precision, features.origin,
                            vertex_index_offset, normal_index_offset);
    std::tie(vertex_index_offset, normal_index_offset) =
        branch_point_mesh.EmitObj(os, kBranchPointGlow,
                                  precision, features.origin,
                                  vertex_index_offset, normal_index_offset);

    std::tie(vertex_index_offset, normal_index_offset) =
        grayed_asphalt_mesh.EmitObj(os, kGrayedBlandAsphalt,
                                    precision, features.origin,
                                    vertex_index_offset, normal_index_offset);
    std::tie(vertex_index_offset, normal_index_offset) =
        grayed_lane_mesh.EmitObj(os, kGrayedLaneHaze,
                                 precision, features.origin,
                                 vertex_index_offset, normal_index_offset);
    std::tie(vertex_index_offset, normal_index_offset) =
        grayed_marker_mesh.EmitObj(os, kGrayedMarkerPaint,
                                   precision, features.origin,
                                   vertex_index_offset, normal_index_offset);

    std::tie(vertex_index_offset, normal_index_offset) =
        h_bounds_mesh.EmitObj(os, kHBoundsHaze,
                              precision, features.origin,
                              vertex_index_offset, normal_index_offset);
  }

  // Create the MTL file referenced by the OBJ file.
  {
    std::ofstream os(dirpath + "/" + mtl_filename, std::ios::binary);
    fmt::print(os,
               R"X(# GENERATED BY maliput::utility::GenerateObjFile()
#
# DON'T BE A HERO.  Do not edit by hand.

newmtl {}
Ka 0.8 0.8 0.0
Kd 1.0 1.0 0.0
Ks 1.0 1.0 0.5
Ns 10.0
illum 2
d 0.5

newmtl {}
Ka 0.1 0.1 0.1
Kd 0.2 0.2 0.2
Ks 0.3 0.3 0.3
Ns 10.0
illum 2

newmtl {}
Ka 0.9 0.9 0.9
Kd 0.9 0.9 0.9
Ks 0.9 0.9 0.9
Ns 10.0
illum 2
d 0.20

newmtl {}
Ka 0.0 0.0 1.0
Kd 0.0 0.0 1.0
Ks 0.0 0.0 1.0
Ns 10.0
illum 2
d 0.80

newmtl {}
Ka 0.8 0.8 0.0
Kd 1.0 1.0 0.0
Ks 1.0 1.0 0.5
Ns 10.0
illum 2
d 0.1

newmtl {}
Ka 0.1 0.1 0.1
Kd 0.2 0.2 0.2
Ks 0.3 0.3 0.3
Ns 10.0
illum 2
d 0.10

newmtl {}
Ka 0.9 0.9 0.9
Kd 0.9 0.9 0.9
Ks 0.9 0.9 0.9
Ns 10.0
illum 2
d 0.10

newmtl {}
Ka 0.0 0.0 1.0
Kd 0.0 0.0 1.0
Ks 0.0 0.0 1.0
Ns 10.0
illum 2
d 0.20
)X",
               kMarkerPaint, kBlandAsphalt, kLaneHaze, kBranchPointGlow,
               kGrayedMarkerPaint, kGrayedBlandAsphalt, kGrayedLaneHaze,
               kHBoundsHaze);
  }
}


}  // namespace utility
}  // namespace maliput
}  // namespace drake
