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

#include "spdlog/fmt/ostr.h"

#include "drake/automotive/maliput/api/junction.h"
#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/api/segment.h"
#include "drake/common/drake_assert.h"

namespace drake {
namespace maliput {
namespace utility {

namespace {

// A container for a set of unique objects which keeps track of the original
// insertion order.  Its primary purpose is to assign a stable unique index
// to each element at time of insertion.
//
// @tparam T  the inserted element type
// @tparam Hash  a hasher suitable for std::unordered_map (e.g., std::hash)
// @tparam KeyEqual  an equivalence relation suitable for std::unordered_map
//                   (e.g., std::equal_to)
template <class T, class Hash, class KeyEqual>
class UniqueIndexer {
 public:
  // Creates an empty UniqueIndexer.
  UniqueIndexer() {}

  // Pushes @p thing onto the back of this container, and returns the unique
  // index for @p thing.  If @p thing has already been added to the container,
  // then this simply returns the original index for @p thing.
  int push_back(const T& thing) {
    auto mi = map_.find(thing);
    if (mi != map_.end()) {
      return mi->second;
    }
    const int index = vector_.size();
    auto it = map_.emplace(thing, index).first;
    vector_.push_back(&(it->first));
    return index;
  }

  // Returns a vector of all elements added to this container.  The index of
  // any element in the vector equals the index generated when the element
  // was added to the container.
  const std::vector<const T*>& vector() const { return vector_; }

 private:
  std::unordered_map<T, int, Hash, KeyEqual> map_;
  std::vector<const T*> vector_;
};


// A GEO-space (world-frame) vertex.
class GeoVertex {
 public:
  // A hasher operation suitable for std::unordered_map.
  struct Hash {
    size_t operator()(const GeoVertex& gv) const {
      const size_t hx(std::hash<double>()(gv.v().x));
      const size_t hy(std::hash<double>()(gv.v().y));
      const size_t hz(std::hash<double>()(gv.v().z));
      return hx ^ (hy << 1) ^ (hz << 2);
    }
  };

  // An equivalence operation suitable for std::unordered_map.
  struct Equiv {
    bool operator()(const GeoVertex& lhs, const GeoVertex& rhs) const {
      return ((lhs.v().x == rhs.v().x) &&
              (lhs.v().y == rhs.v().y) &&
              (lhs.v().z == rhs.v().z));
    }
  };

  GeoVertex() {}

  explicit GeoVertex(const api::GeoPosition& v) : v_(v) {}

  const api::GeoPosition& v() const { return v_; }

 private:
  api::GeoPosition v_;
};


// A GEO-space (world-frame) normal vector.
class GeoNormal {
 public:
  // A hasher operation suitable for std::unordered_map.
  struct Hash {
    size_t operator()(const GeoNormal& gn) const {
      const size_t hx(std::hash<double>()(gn.n().x));
      const size_t hy(std::hash<double>()(gn.n().y));
      const size_t hz(std::hash<double>()(gn.n().z));
      return hx ^ (hy << 1) ^ (hz << 2);
    }
  };

  // An equivalence operation suitable for std::unordered_map.
  struct Equiv {
    bool operator()(const GeoNormal& lhs, const GeoNormal& rhs) const {
      return ((lhs.n().x == rhs.n().x) &&
              (lhs.n().y == rhs.n().y) &&
              (lhs.n().z == rhs.n().z));
    }
  };

  GeoNormal() {}

  // Construct a GeoNormal as the vector from @p v0 to @p v1.
  GeoNormal(const api::GeoPosition& v0, const api::GeoPosition& v1)
      : n_({v1.x - v0.x, v1.y - v0.y, v1.z - v0.z}) {}

  const api::GeoPosition& n() const { return n_; }

 private:
  api::GeoPosition n_;
};


// A GEO-space (world-frame) face:  a sequence of vertices with corresponding
// normals.
class GeoFace {
 public:
  GeoFace() {}

  GeoFace(const std::vector<GeoVertex>& vertices,
          const std::vector<GeoNormal>& normals)
      : vertices_(vertices), normals_(normals) {
    DRAKE_DEMAND(vertices.size() == normals.size());
  }

  void push_vn(const GeoVertex& vertex, const GeoNormal& normal) {
    vertices_.push_back(vertex);
    normals_.push_back(normal);
  }

  const std::vector<GeoVertex>& vertices() const { return vertices_; }

  const std::vector<GeoNormal>& normals() const { return normals_; }

 private:
  std::vector<GeoVertex> vertices_;
  std::vector<GeoNormal> normals_;
};


// A face --- a sequence of vertices with normals --- in which the
// vertices and normals are represented by integer indices into some
// other container/dictionary.
class IndexFace {
 public:
  struct Vertex {
    Vertex(int vertex_index_in, int normal_index_in)
        : vertex_index(vertex_index_in),
          normal_index(normal_index_in) {}

    int vertex_index{};
    int normal_index{};
  };

  void push_vertex(int vertex_index, int normal_index) {
    vertices_.emplace_back(vertex_index, normal_index); }

  const std::vector<Vertex>& vertices() const { return vertices_; }

 private:
  std::vector<Vertex> vertices_;
};


// A GEO-space (world-frame) mesh:  a collection of GeoFaces.
class GeoMesh {
 public:
  GeoMesh() {}

  void PushFace(const GeoFace& geo_face) {
    IndexFace face;
    for (size_t gi = 0; gi < geo_face.vertices().size(); ++gi) {
      int vi = vertices_.push_back(geo_face.vertices()[gi]);
      int ni = normals_.push_back(geo_face.normals()[gi]);
      face.push_vertex(vi, ni);
    }
    faces_.push_back(face);
  }

  // Emits the mesh as Wavefront OBJ elements to @p os.  @p material is the
  // name of an MTL-defined material to describe visual properties of the mesh.
  //
  // If other meshes have already been emitted to stream @p os, then
  // @p vertex_index_offset and @p normal_index_offset must correspond to the
  // number of previously emitted vertices and normals respectively.
  // Conveniently, EmitObj() returns a tuple of (vertex_index_offset,
  // normal_index_offset) which can be chained into a succeeding call to
  // EmitObj().
  std::tuple<int, int>
  EmitObj(std::ostream& os, const std::string& material,
          int vertex_index_offset, int normal_index_offset) {
    os << "# Vertices\n";
    for (const GeoVertex* gv : vertices_.vector()) {
      os << "v " << gv->v().x << " " << gv->v().y << " " << gv->v().z << "\n";
    }
    os << "# Normals\n";
    for (const GeoNormal* gn : normals_.vector()) {
      os << "vn " << gn->n().x << " " << gn->n().y << " " << gn->n().z << "\n";
    }
    os << "\n"
       << "# Faces\n";
    if (!material.empty()) {
      os << "usemtl " << material << "\n";
    }
    for (const IndexFace& f : faces_) {
      os << "f";
      for (const IndexFace::Vertex& ifv : f.vertices()) {
        os << " " << (ifv.vertex_index + 1 + vertex_index_offset)
           << "//" << (ifv.normal_index + 1 + normal_index_offset);
      }
      os << "\n";
    }
    return std::make_tuple(vertex_index_offset + vertices_.vector().size(),
                           normal_index_offset + normals_.vector().size());
  }

 private:
  UniqueIndexer<GeoVertex, GeoVertex::Hash, GeoVertex::Equiv> vertices_;
  UniqueIndexer<GeoNormal, GeoNormal::Hash, GeoNormal::Equiv> normals_;
  std::vector<IndexFace> faces_;
};


// A LANE-space face: a sequence of vertices expressed in the (s,r,h)
// coordinates of an api::Lane (which is not referenced here).  Each
// vertex has an implicit unit-length normal vector in the +h
// direction normal to the road surface.
class SrhFace {
 public:
  SrhFace(const std::initializer_list<api::LanePosition> srh) : v_(srh) {
    // TODO(maddog@tri.global) Provide for explicit normals if we ever
    // consider faces which are not parallel to the road surface.
    for (const api::LanePosition& vertex : v_) {
      DRAKE_DEMAND(vertex.h == v_[0].h);
    }
  }

  const std::vector<api::LanePosition>& v() const { return v_; }

  // Given a @p lane, calculates the corresponding GeoFace.
  GeoFace ToGeoFace(const api::Lane* lane) const {
    GeoFace geo_face;
    for (const api::LanePosition& srh : v_) {
      api::GeoPosition v0(lane->ToGeoPosition(srh));
      api::GeoPosition v1(lane->ToGeoPosition({srh.s, srh.r, srh.h + 1.}));
      geo_face.push_vn(GeoVertex(v0), GeoNormal(v0, v1));
    }
    return geo_face;
  }

 private:
  std::vector<api::LanePosition> v_;
};


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
// @param h_offset  h value of each vertex (height above road surface)
void CoverLaneWithQuads(GeoMesh* mesh, const api::Lane* lane,
                        double grid_unit,
                        bool use_driveable_bounds,
                        double h_offset) {
  const double s_max = lane->length();
  for (double s0 = 0; s0 < s_max; s0 += grid_unit) {
    double s1 = s0 + grid_unit;
    if (s1 > s_max) { s1 = s_max; }

    const api::RBounds rb0 = use_driveable_bounds ?
        lane->driveable_bounds(s0) : lane->lane_bounds(s0);
    const api::RBounds rb1 = use_driveable_bounds ?
        lane->driveable_bounds(s1) : lane->lane_bounds(s1);

    // Left side of lane (r >= 0).
    {
      double r00 = 0.;
      double r10 = 0.;
      while ((r00 < rb0.r_max) && (r10 < rb1.r_max)) {
        const double r01 = std::min(r00 + grid_unit, rb0.r_max);
        const double r11 = std::min(r10 + grid_unit, rb1.r_max);
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
            {s0, r01, h_offset}});
        mesh->PushFace(srh_face.ToGeoFace(lane));

        r00 += grid_unit;
        r10 += grid_unit;
      }
    }
    // Right side of lane (r <= 0).
    {
      double r00 = 0.;
      double r10 = 0.;
      while ((r00 > rb0.r_min) && (r10 > rb1.r_min)) {
        const double r01 = std::max(r00 - grid_unit, rb0.r_min);
        const double r11 = std::max(r10 - grid_unit, rb1.r_min);
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
            {s1, r10, h_offset}});
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

  const double s_max = lane->length();
  for (double s0 = 0; s0 < s_max; s0 += grid_unit) {
    double s1 = s0 + grid_unit;
    if (s1 > s_max) { s1 = s_max; }

    api::RBounds rb0 = lane->lane_bounds(s0);
    api::RBounds rb1 = lane->lane_bounds(s1);

    // Left side of lane.
    {
      SrhFace srh_face({
          {s0, rb0.r_max - half_stripe, h_offset},
          {s1, rb1.r_max - half_stripe, h_offset},
          {s1, rb1.r_max + half_stripe, h_offset},
          {s0, rb0.r_max + half_stripe, h_offset}});
      mesh->PushFace(srh_face.ToGeoFace(lane));
    }
    // Right side of lane.
    {
      SrhFace srh_face({
          {s0, rb0.r_min - half_stripe, h_offset},
          {s1, rb1.r_min - half_stripe, h_offset},
          {s1, rb1.r_min + half_stripe, h_offset},
          {s0, rb0.r_min + half_stripe, h_offset}});
      mesh->PushFace(srh_face.ToGeoFace(lane));
    }
  }
}


// Adds faces to @p mesh which draw a simple triangular arrow in the
// LANE-space of @p lane.  The width of the arrow is fixed at 80% of
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

  const double rl_size = rb0.r_max * kRelativeWidth;
  const double rr_size = -rb0.r_min * kRelativeWidth;
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
            {s0, r01, h_offset}});
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
          {s0, r00 + rl_unit, h_offset}});
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
            {s1, r10, h_offset}});
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
          {s1, r10, h_offset}});
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
  const double max_length = 0.3 * lane->length();
  // Arrows are sized relative to their respective ends.
  const api::RBounds start_rb = lane->lane_bounds(0.);
  const double start_s_size = std::min(max_length,
                                       (start_rb.r_max - start_rb.r_min));

  const api::RBounds finish_rb = lane->lane_bounds(lane->length());
  const double finish_s_size = std::min(max_length,
                                        (finish_rb.r_max - finish_rb.r_min));

  DrawLaneArrow(mesh, lane, grid_unit, 0., start_s_size, h_offset);
  DrawLaneArrow(mesh, lane, grid_unit,
                lane->length() - finish_s_size, finish_s_size, h_offset);
}


// Calculates an appropriate grid-unit size for @p lane.
double PickGridUnit(const api::Lane* lane,
                    double max_size, double min_resolution) {
  double result = max_size;
  const api::RBounds rb0 = lane->lane_bounds(0.);
  const api::RBounds rb1 = lane->lane_bounds(lane->length());
  result = std::min(result, (rb0.r_max - rb0.r_min) / min_resolution);
  result = std::min(result, (rb1.r_max - rb1.r_min) / min_resolution);
  result = std::min(result, lane->length() / min_resolution);
  return result;
}

}  // namespace


void GenerateObjFile(const api::RoadGeometry* rg,
                     const std::string& dirpath,
                     const std::string& fileroot,
                     const ObjFeatures& features) {
  GeoMesh asphalt_mesh;
  GeoMesh lane_mesh;
  GeoMesh marker_mesh;

  // Walk the network.
  for (int ji = 0; ji < rg->num_junctions(); ++ji) {
    const api::Junction* junction = rg->junction(ji);
    for (int si = 0; si < junction->num_segments(); ++si) {
      const api::Segment* segment = junction->segment(si);
      // Lane 0 should be as good as any other for driveable-bounds.
      CoverLaneWithQuads(&asphalt_mesh, segment->lane(0),
                         PickGridUnit(segment->lane(0),
                                      features.max_grid_unit,
                                      features.min_grid_resolution),
                         true, 0.);
      for (int li = 0; li < segment->num_lanes(); ++li) {
        const api::Lane* lane = segment->lane(li);
        const double grid_unit = PickGridUnit(lane,
                                              features.max_grid_unit,
                                              features.min_grid_resolution);
        if (features.draw_lane_haze) {
          CoverLaneWithQuads(&lane_mesh, lane, grid_unit,
                             false,
                             features.lane_haze_elevation);
        }
        if (features.draw_stripes) {
          StripeLaneBounds(&marker_mesh, lane, grid_unit,
                           features.stripe_elevation,
                           features.stripe_width);
        }
        if (features.draw_arrows) {
          MarkLaneEnds(&marker_mesh, lane, grid_unit,
                       features.arrow_elevation);
        }
      }
    }
  }

  const std::string kLaneHaze("lane_haze");
  const std::string kMarkerPaint("marker_paint");
  const std::string kBlandAsphalt("bland_asphalt");

  const std::string obj_filename = fileroot + ".obj";
  const std::string mtl_filename = fileroot + ".mtl";

  // Create the requested OBJ file.
  {
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
                             vertex_index_offset, normal_index_offset);
    std::tie(vertex_index_offset, normal_index_offset) =
        lane_mesh.EmitObj(os, kLaneHaze,
                          vertex_index_offset, normal_index_offset);
    std::tie(vertex_index_offset, normal_index_offset) =
        marker_mesh.EmitObj(os, kMarkerPaint,
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
)X",
               kMarkerPaint, kBlandAsphalt, kLaneHaze);
  }
}


}  // namespace utility
}  // namespace maliput
}  // namespace drake
