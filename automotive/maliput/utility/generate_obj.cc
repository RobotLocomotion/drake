#include "drake/automotive/maliput/utility/generate_obj.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <functional>
#include <initializer_list>
#include <queue>
#include <set>
#include <tuple>
#include <unordered_map>
#include <vector>

#include <Eigen/Geometry>
#include "fmt/ostream.h"

#include "drake/automotive/maliput/api/branch_point.h"
#include "drake/automotive/maliput/api/junction.h"
#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/api/segment.h"
#include "drake/common/drake_assert.h"
#include "drake/common/hash.h"

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


// A world frame vertex.
class GeoVertex {
 public:
  // A hasher operation suitable for std::unordered_map.
  using Hash = drake::DefaultHash;

  // An equivalence operation suitable for std::unordered_map.
  struct Equiv {
    bool operator()(const GeoVertex& lhs, const GeoVertex& rhs) const {
      return (lhs.v().xyz() == rhs.v().xyz());
    }
  };

  GeoVertex() {}

  explicit GeoVertex(const api::GeoPosition& v) : v_(v) {}

  const api::GeoPosition& v() const { return v_; }

  /// Implements the @ref hash_append concept.
  template <class HashAlgorithm>
  friend void hash_append(
      HashAlgorithm& hasher, const GeoVertex& item) noexcept {
    using drake::hash_append;
    hash_append(hasher, item.v_.x());
    hash_append(hasher, item.v_.y());
    hash_append(hasher, item.v_.z());
  }

 private:
  api::GeoPosition v_;
};


// A world frame normal vector.
class GeoNormal {
 public:
  // A hasher operation suitable for std::unordered_map.
  using Hash = drake::DefaultHash;

  // An equivalence operation suitable for std::unordered_map.
  struct Equiv {
    bool operator()(const GeoNormal& lhs, const GeoNormal& rhs) const {
      return (lhs.n().xyz() == rhs.n().xyz());
    }
  };

  GeoNormal() {}

  explicit GeoNormal(const api::GeoPosition& n) : n_(n) {}

  const api::GeoPosition& n() const { return n_; }

  /// Implements the @ref hash_append concept.
  template <class HashAlgorithm>
  friend void hash_append(
      HashAlgorithm& hasher, const GeoNormal& item) noexcept {
    using drake::hash_append;
    hash_append(hasher, item.n_.x());
    hash_append(hasher, item.n_.y());
    hash_append(hasher, item.n_.z());
  }

 private:
  api::GeoPosition n_;
};


// A world frame face:  a sequence of vertices with corresponding normals.
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


// A world frame mesh:  a collection of GeoFaces.
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

  void AddFacesFrom(const GeoMesh& other_mesh) {
    const std::vector<IndexFace>& other_faces = other_mesh.faces();
    const std::vector<const GeoVertex*>& other_vertices = other_mesh.vertices();
    const std::vector<const GeoNormal*>& other_normals = other_mesh.normals();
    for (const IndexFace& other_face : other_faces) {
      IndexFace face;
      const std::vector<IndexFace::Vertex>&
          other_face_vertices = other_face.vertices();
      for (size_t i = 0; i < other_face_vertices.size(); ++i) {
        const IndexFace::Vertex& other_face_vertex = other_face_vertices[i];
        int vi = vertices_.push_back(*other_vertices[
            other_face_vertex.vertex_index]);
        int ni = normals_.push_back(*other_normals[
            other_face_vertex.normal_index]);
        face.push_vertex(vi, ni);
      }
      faces_.push_back(face);
    }
  }

  // Emits the mesh as Wavefront OBJ elements to @p os.  @p material is the
  // name of an MTL-defined material to describe visual properties of the mesh.
  // @p precision specifies the fixed-point precision (number of digits after
  // the decimal point) for vertex and normal coordinate values.  @p origin
  // specifies a world-frame origin for vertex coordinates (i.e., the emitted
  // values for a vertex at `(x,y,z)` will be `(x,y,z) - origin`).
  //
  // If other meshes have already been emitted to stream @p os, then
  // @p vertex_index_offset and @p normal_index_offset must correspond to the
  // number of previously emitted vertices and normals respectively.
  // Conveniently, EmitObj() returns a tuple of (vertex_index_offset,
  // normal_index_offset) which can be chained into a succeeding call to
  // EmitObj().
  std::tuple<int, int>
  EmitObj(std::ostream& os, const std::string& material,
          int precision, const api::GeoPosition& origin,
          int vertex_index_offset, int normal_index_offset) {
    if (faces_.empty()) {
      // Short-circuit if there is nothing to draw.
      return std::make_tuple(vertex_index_offset, normal_index_offset);
    }

    // NOLINTNEXTLINE(build/namespaces)  Usage documented by fmt library.
    using namespace fmt::literals;
    fmt::print(os, "# Vertices\n");
    for (const GeoVertex* gv : vertices_.vector()) {
      fmt::print(os, "v {x:.{p}f} {y:.{p}f} {z:.{p}f}\n",
                 "x"_a = (gv->v().x() - origin.x()),
                 "y"_a = (gv->v().y() - origin.y()),
                 "z"_a = (gv->v().z() - origin.z()),
                 "p"_a = precision);
    }
    fmt::print(os, "# Normals\n");
    for (const GeoNormal* gn : normals_.vector()) {
      fmt::print(os, "vn {x:.{p}f} {y:.{p}f} {z:.{p}f}\n",
                 "x"_a = gn->n().x(), "y"_a = gn->n().y(), "z"_a = gn->n().z(),
                 "p"_a = precision);
    }
    fmt::print(os, "\n");
    fmt::print(os, "# Faces\n");
    if (!material.empty()) {
      fmt::print(os, "usemtl {}\n", material);
    }
    for (const IndexFace& f : faces_) {
      fmt::print(os, "f");
      for (const IndexFace::Vertex& ifv : f.vertices()) {
        fmt::print(os, " {}//{}",
                   (ifv.vertex_index + 1 + vertex_index_offset),
                   (ifv.normal_index + 1 + normal_index_offset));
      }
      fmt::print(os, "\n");
    }
    return std::make_tuple(vertex_index_offset + vertices_.vector().size(),
                           normal_index_offset + normals_.vector().size());
  }

  const std::vector<const GeoVertex*>& vertices() const {
    return vertices_.vector();
  }

  const std::vector<const GeoNormal*>& normals() const {
    return normals_.vector();
  }

  const std::vector<IndexFace>& faces() const { return faces_; }

 private:
  UniqueIndexer<GeoVertex, GeoVertex::Hash, GeoVertex::Equiv> vertices_;
  UniqueIndexer<GeoNormal, GeoNormal::Hash, GeoNormal::Equiv> normals_;
  std::vector<IndexFace> faces_;
};


// A `Lane`-frame face: a sequence of vertices expressed in the (s,r,h)
// coordinates of an api::Lane (which is not referenced here).  Each
// vertex has a normal vector also expressed in the `Lane`-frame.
class SrhFace {
 public:
  SrhFace(const std::initializer_list<api::LanePosition> vertices,
          const api::LanePosition& normal)
      : vertices_(vertices),
        normal_(normal) {}

  // Given a @p lane, calculates the corresponding GeoFace.
  GeoFace ToGeoFace(const api::Lane* lane) const {
    GeoFace geo_face;
    for (const api::LanePosition& srh : vertices_) {
      api::GeoPosition xyz(lane->ToGeoPosition(srh));
      api::GeoPosition n = api::GeoPosition::FromXyz(
          lane->GetOrientation(srh).quat() * normal_.srh());
      geo_face.push_vn(GeoVertex(xyz), GeoNormal(n));
    }
    return geo_face;
  }

 private:
  std::vector<api::LanePosition> vertices_;
  api::LanePosition normal_;
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
// @param elevation a function taking `(s, r)` as parameters and returning
//        the corresponding elevation `h`, to yield a quad vertex `(s, r, h)`
void CoverLaneWithQuads(
    GeoMesh* mesh, const api::Lane* lane,
    double grid_unit, bool use_driveable_bounds,
    const std::function<double(double, double)>& elevation) {
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

  const double s_max = lane->length();
  for (double s0 = 0; s0 < s_max; s0 += grid_unit) {
    double s1 = s0 + grid_unit;
    if (s1 > s_max) { s1 = s_max; }

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

// Index for a directed edge in a GeoMesh.
struct DirectedEdgeIndex {
  int start_vertex_index;  //< Index of start vertex.
  int end_vertex_index;  //< Index of end vertex.
};

bool operator==(const DirectedEdgeIndex& lhs, const DirectedEdgeIndex& rhs) {
  return (lhs.start_vertex_index == rhs.start_vertex_index
          && lhs.end_vertex_index == rhs.end_vertex_index);
}

/// Implements the @ref hash_append concept.
template <class HashAlgorithm>
void hash_append(
    HashAlgorithm& hasher, const DirectedEdgeIndex& item) noexcept {
  using drake::hash_append;
  hash_append(hasher, item.start_vertex_index);
  hash_append(hasher, item.end_vertex_index);
}

// Index for a face edge in a GeoMesh.
struct FaceEdgeIndex {
  int face_index;  //< Index of the face the edge belongs to.
  int edge_index;  //< Index of the edge, corresponding to its
                   //  start vertex index in the face vertices
                   //  sequence (assumed to be defined in a
                   //  counter-clockwise fashion).
};

bool operator==(const FaceEdgeIndex& lhs, const FaceEdgeIndex& rhs) {
  return (lhs.face_index == rhs.face_index &&
          lhs.edge_index == rhs.edge_index);
}

bool operator!=(const FaceEdgeIndex& lhs, const FaceEdgeIndex& rhs) {
  return !(lhs == rhs);
}

// The inverse of the mapping from face edges indices to their
// associated directed edge indices.
using InverseFaceEdgeMap = std::unordered_map<
  DirectedEdgeIndex, FaceEdgeIndex, drake::DefaultHash>;

// Computes the inverse of the mapping from face edges indices to their
// associated directed edge indices for the given @p faces collection.
// @pre Mapping from directed edges to face edges is 1-to-1 (or in other
//      words, any given pair of vertices can be shared by two faces at
//      most).
// @warning If any of the preconditions is not met, this function will
//          abort execution.
InverseFaceEdgeMap
ComputeInverseFaceEdgeMap(const std::vector<IndexFace>& faces) {
  InverseFaceEdgeMap inverse_face_edge_map;
  const int faces_count = static_cast<int>(faces.size());
  for (int face_index = 0; face_index < faces_count; ++face_index) {
    const IndexFace& face = faces[face_index];
    const std::vector<IndexFace::Vertex>& face_vertices = face.vertices();
    const int face_vertex_count = static_cast<int>(face_vertices.size());
    for (int start_vertex_index = 0;
         start_vertex_index < face_vertex_count; ++start_vertex_index) {
      int end_vertex_index = (start_vertex_index + 1) % face_vertex_count;
      const DirectedEdgeIndex global_edge{
        face_vertices[start_vertex_index].vertex_index,
        face_vertices[end_vertex_index].vertex_index};
      DRAKE_DEMAND(inverse_face_edge_map.count(global_edge) == 0);
      inverse_face_edge_map[global_edge] = {face_index, start_vertex_index};
    }
  }
  return inverse_face_edge_map;
}

// A mapping providing all adjacent faces to each
// face in a given GeoMesh, along with the edge
// these share.
using FaceAdjacencyMap = std::unordered_map<
  int, std::vector<FaceEdgeIndex>>;

// Computes a mapping providing all adjacent faces to each
// face in @p faces, along with the edge these share.
// @pre Any given pair of vertices is shared by two faces at most.
// @warning If any of the preconditions is not met, this function will
//          abort execution.
FaceAdjacencyMap
ComputeFaceAdjacencyMap(const std::vector<IndexFace>& faces) {
  FaceAdjacencyMap adjacent_faces_map;

  InverseFaceEdgeMap inverse_face_edge_map =
      ComputeInverseFaceEdgeMap(faces);
  for (const auto& entry : inverse_face_edge_map) {
    const DirectedEdgeIndex& edge_index = entry.first;
    const FaceEdgeIndex& face_edge_index = entry.second;
    if (adjacent_faces_map.count(face_edge_index.face_index) == 0) {
      adjacent_faces_map[face_edge_index.face_index].assign(
          faces[face_edge_index.face_index].vertices().size(), {-1, -1});
    }
    const DirectedEdgeIndex reversed_edge_index{
      edge_index.end_vertex_index, edge_index.start_vertex_index};
    if (inverse_face_edge_map.count(reversed_edge_index) > 0) {
      std::vector<FaceEdgeIndex>& adjacent_face_edges =
          adjacent_faces_map[face_edge_index.face_index];
      adjacent_face_edges[face_edge_index.edge_index] =
          inverse_face_edge_map[reversed_edge_index];
    }
  }
  return adjacent_faces_map;
}

template <typename T>
using Hyperplane3 = Eigen::Hyperplane<T, 3>;

// Gets global position of the @p vertex in the given @p mesh.
// @pre Given @p vertex belongs to the @p mesh.
// @warning If any of the preconditions is not met, this function will
//          abort execution.
const Vector3<double>& GetMeshFaceVertexPosition(
    const GeoMesh& mesh, const IndexFace::Vertex& vertex) {
  DRAKE_DEMAND(0 <= vertex.vertex_index);
  const std::vector<const GeoVertex*>& vertices = mesh.vertices();
  DRAKE_DEMAND(vertex.vertex_index < static_cast<int>(vertices.size()));
  return vertices[vertex.vertex_index]->v().xyz();
}

// Gets normal vector of the @p vertex in the given @p mesh.
// @pre Given @p vertex belongs to the @p mesh.
// @warning If any of the preconditions is not met, this function will
//          abort execution.
const Vector3<double>& GetMeshFaceVertexNormal(
    const GeoMesh& mesh, const IndexFace::Vertex& vertex) {
  DRAKE_DEMAND(0 <= vertex.normal_index);
  const std::vector<const GeoNormal*>& normals = mesh.normals();
  DRAKE_DEMAND(vertex.normal_index < static_cast<int>(normals.size()));
  return normals[vertex.normal_index]->n().xyz();
}


// Checks if all the vertices, from @p first to @p last, in the given
// @p mesh lie on the provided @p plane by verifying all of them are
// within one @p tolerance distance, in meters, away from it along
// the line subtended by its normal.
// @pre Given @p vertices belong to the @p mesh.
// @warning If any of the preconditions is not met, this function
//          will abort execution.
// @tparam InputIt An IndexFace::Vertex container iterator type.
template <typename InputIt>
bool DoMeshVerticesLieOnPlane(
    const GeoMesh& mesh, InputIt first, InputIt last,
    const Hyperplane3<double>& plane, double tolerance) {
  return std::all_of(
      first, last, [&mesh, &plane, tolerance](const IndexFace::Vertex& vertex) {
        const Vector3<double>& x = GetMeshFaceVertexPosition(mesh, vertex);
        const Vector3<double>& n = GetMeshFaceVertexNormal(mesh, vertex);
        const double ctheta = std::abs(plane.normal().dot(n.normalized()));
        return (ctheta != 0. && plane.absDistance(x) / ctheta < tolerance);
      });
}


// Checks if the @p face in the given @p mesh is coplanar with the
// given @p plane, by verifying if all @p face vertices are within
// one @p tolerance distance, in meters, from it.
// @pre Given @p face belongs to the @p mesh.
// @warning If any of the preconditions is not met, this function
//          will abort execution.
bool IsMeshFaceCoplanarWithPlane(const GeoMesh& mesh,
                                 const IndexFace& face,
                                 const Hyperplane3<double>& plane,
                                 double tolerance) {
  const std::vector<IndexFace::Vertex>& face_vertices = face.vertices();
  return DoMeshVerticesLieOnPlane(mesh, face_vertices.begin(),
                                  face_vertices.end(), plane,
                                  tolerance);
}


// Checks if the @p face in the given @p mesh is planar, by verifying
// all @p face vertices lie on a plane using the given @p tolerance
// (see DoMeshVerticesLieOnPlane()). Said plane, built out of the
// first vertex position and normal in the @p face, is returned as
// @p plane.
// @pre Given @p face belongs to the @p mesh.
// @warning If any of the preconditions is not met, this function
//          will abort execution.
bool IsMeshFacePlanar(const GeoMesh& mesh, const IndexFace& face,
                      double tolerance, Hyperplane3<double>* plane) {
  DRAKE_DEMAND(plane != nullptr);
  const std::vector<IndexFace::Vertex>& face_vertices = face.vertices();
  DRAKE_DEMAND(face_vertices.size() >= 3);
  const Vector3<double>& x0 = GetMeshFaceVertexPosition(mesh, face_vertices[0]);
  const Vector3<double>& n  = GetMeshFaceVertexNormal(mesh, face_vertices[0]);
  *plane =  Hyperplane3<double>(n.normalized(), x0);
  return DoMeshVerticesLieOnPlane(mesh, face_vertices.begin() + 1,
                                  face_vertices.end(), *plane,
                                  tolerance);
}


// Aggregates all coplanar faces adjacent to the referred face in the @p mesh.
// @param mesh Mesh where faces are to be found.
// @param start_face_index Index of the face to start aggregation with.
// @param adjacent_faces_map Map of adjacent faces associated with the
//                           given @p mesh.
// @param tolerance For coplanarity checks, in meters. See IsMeshFacePlanar()
//                  and IsMeshFaceCoplanarWithPlane() functions for further
//                  details.
// @param visited_faces_indices The indices of the faces visited so far.
// @returns The indices of the adjacent coplanar faces found.
// @pre Given @p face_index is valid for the given @p mesh.
// @pre Given @p tolerance is a positive real number.
// @pre Given @p visited_faces_indices collection is not nullptr.
// @pre Given @p face_index has not been visited yet
//      (i.e. visited_faces_indices.count(face_index) == 0).
// @post All adjacent coplanar faces found are marked as visited.
// @warning If any of the preconditions is not met, this function
//          will abort execution.
std::set<int> AggregateAdjacentCoplanarMeshFaces(
    const GeoMesh& mesh, int start_face_index,
    const FaceAdjacencyMap& adjacent_faces_map,
    double tolerance, std::set<int>* visited_faces_indices) {
  DRAKE_DEMAND(0 <= start_face_index);
  const std::vector<IndexFace>& faces = mesh.faces();
  DRAKE_DEMAND(start_face_index < static_cast<int>(faces.size()));
  DRAKE_DEMAND(tolerance > 0.);
  DRAKE_DEMAND(visited_faces_indices != nullptr);
  DRAKE_DEMAND(visited_faces_indices->count(start_face_index) == 0);

  // Traverse adjacent faces, collecting coplanar ones.
  std::set<int> mergeable_faces_indices{start_face_index};
  Hyperplane3<double> start_face_plane;
  if (IsMeshFacePlanar(mesh, faces[start_face_index],
                       tolerance, &start_face_plane)) {
    std::queue<int> faces_indices_to_visit({start_face_index});
    while (!faces_indices_to_visit.empty()) {
      int face_index = faces_indices_to_visit.front();
      const std::vector<FaceEdgeIndex>&
          adjacent_face_edges = adjacent_faces_map.at(face_index);
      const int face_edge_count = adjacent_face_edges.size();
      for (int edge_index = 0; edge_index < face_edge_count; ++edge_index) {
        const int adjacent_face_index =
            adjacent_face_edges[edge_index].face_index;
        if (adjacent_face_index == -1) continue;
        if (mergeable_faces_indices.count(adjacent_face_index) > 0) {
          continue;
        }
        if (visited_faces_indices->count(adjacent_face_index) > 0) {
          continue;
        }
        if (IsMeshFaceCoplanarWithPlane(mesh, faces[adjacent_face_index],
                                        start_face_plane, tolerance)) {
          mergeable_faces_indices.insert(adjacent_face_index);
          faces_indices_to_visit.push(adjacent_face_index);
        }
      }
      visited_faces_indices->insert(face_index);
      faces_indices_to_visit.pop();
    }
  } else {
    visited_faces_indices->insert(start_face_index);
  }
  return mergeable_faces_indices;
}


// Finds the index to the first outer face edge in the given
// @p simply_connected_faces_indices.
// @param simply_connected_faces_indices Indices of the faces whose
//                                       outer face edge is to be found.
// @param adjacent_faces_map Mapping of adjacent faces for the faces
//                           referred by @p simply_connected_faces_indices.
// @pre The union of the all the faces referred by the given
//      @p simply_connected_faces_indices expands a simply
//      connected region (i.e. with no holes).
FaceEdgeIndex
FindOuterFaceEdgeIndex(const std::set<int>& simply_connected_faces_indices,
                       const FaceAdjacencyMap& adjacent_faces_map) {
  // Goes over each face in the simply connected region, looking
  // for the first face edge that's not adjacent to a face within
  // the same region.
  for (int face_index : simply_connected_faces_indices) {
    const std::vector<FaceEdgeIndex>& adjacent_face_edges =
        adjacent_faces_map.at(face_index);
    auto it = std::find_if(
        adjacent_face_edges.begin(), adjacent_face_edges.end(),
        [&simply_connected_faces_indices](const FaceEdgeIndex& face_edge) {
          return (simply_connected_faces_indices.count(
              face_edge.face_index) == 0);
        });
    if (it != adjacent_face_edges.end()) {
      const int edge_index =
          std::distance(adjacent_face_edges.begin(), it);
      return {face_index, edge_index};
    }
  }
  return {-1, -1};
}


// Index of a face vertex in a GeoMesh.
struct FaceVertexIndex {
  int face_index;  //< Index of the face the vertex belongs to.
  int vertex_index;  //< Index of the face vertex.
};


// Computes the contour of the simply connected region that all the faces
// referred by the given @p simply_connected_faces_indices expand.
// @param simply_connected_faces_indices Indices of the faces whose outer face
//                                       edge is to be found.
// @param adjacent_faces_map Mapping of adjacent faces for the faces referred
//                           in @p simply_connected_faces_indices.
// @returns Face vertices as a counter-clockwise contour.
// @pre The union of the all the faces referred by the given
//      @p simply_connected_faces_indices expands a simply
//      connected region (i.e. with no holes).
std::vector<FaceVertexIndex> ComputeMeshFacesContour(
    const std::set<int>& simply_connected_faces_indices,
    const FaceAdjacencyMap& adjacent_faces_map) {
  std::vector<FaceVertexIndex> faces_contour_indices;
  // Finds first outer edge index for the given simply connected region.
  const FaceEdgeIndex first_outer_face_edge_index = FindOuterFaceEdgeIndex(
      simply_connected_faces_indices, adjacent_faces_map);
  DRAKE_DEMAND(simply_connected_faces_indices.count(
      first_outer_face_edge_index.face_index) > 0);

  // Follows simply connected region contour by crawling through
  // adjacent faces.
  FaceEdgeIndex outer_face_edge_index = first_outer_face_edge_index;
  do {
    const std::vector<FaceEdgeIndex>& adjacent_outer_face_edges =
        adjacent_faces_map.at(outer_face_edge_index.face_index);
    const FaceEdgeIndex& other_face_edge_index =
        adjacent_outer_face_edges[outer_face_edge_index.edge_index];
    if (simply_connected_faces_indices.count(
            other_face_edge_index.face_index) == 0) {
      faces_contour_indices.push_back({outer_face_edge_index.face_index,
                                       outer_face_edge_index.edge_index});
    } else {
      outer_face_edge_index = other_face_edge_index;
    }
    const int outer_face_edge_count =
        adjacent_faces_map.at(outer_face_edge_index.face_index).size();
    outer_face_edge_index.edge_index =
        (outer_face_edge_index.edge_index + 1) % outer_face_edge_count;
  } while (outer_face_edge_index != first_outer_face_edge_index);
  return faces_contour_indices;
}


// Gets the face vertex in the @p mesh referred by the given
// @p face_vertex_index.
// @pre Given @p face_vertex_index refers to a valid vertex
//      in the @p mesh.
// @warning If any of the preconditions is not met, this function
//          will abort execution.
const IndexFace::Vertex& MeshFaceVertexAt(
    const GeoMesh& mesh, const FaceVertexIndex& face_vertex_index) {
  const std::vector<IndexFace>& faces = mesh.faces();
  DRAKE_DEMAND(0 <= face_vertex_index.face_index);
  DRAKE_DEMAND(face_vertex_index.face_index <
               static_cast<int>(faces.size()));
  const std::vector<IndexFace::Vertex>& face_vertices =
      faces[face_vertex_index.face_index].vertices();
  DRAKE_DEMAND(0 <= face_vertex_index.vertex_index);
  DRAKE_DEMAND(face_vertex_index.vertex_index <
               static_cast<int>(face_vertices.size()));
  return face_vertices[face_vertex_index.vertex_index];
}


// Applies the Douglas-Peucker simplification algorithm [1] over the
// given collection of vertices.
//
// - [1] Douglas, D. H., & Peucker, T. K. (1973). Algorithms for the
//       reduction of the number of points required to represent a
//       digitized line or its caricature. Cartographica: The
//       International Journal for Geographic Information and
//       Geovisualization, 10(2), 112–122.
// @param first Iterator at the beginning of the collection.
// @param last Iterator at the end of the collection.
// @param to_vertex A function to retrieve the vertex associated with
//                  an element of the collection (may be a pass-through).
// @param to_edge A function to construct an edge out of a pair of
//                vertices. Said edge MUST support a .distance() method
//                taking a vertex as its sole argument.
// @param tolerance Simplification tolerance, in distance units.
// @param output Output iterator for the simplification result.
// @tparam InputIt Input iterators type.
// @tparam VertexFn Vertex getter function type.
// @tparam EdgeFn Edge building function type.
// @tparam OutputIt  Output iterator type.
template <typename InputIt, typename VertexFn,
          typename EdgeFn, typename OutputIt>
void ApplyDouglasPeuckerSimplification(InputIt first, InputIt last,
                                       VertexFn to_vertex, EdgeFn to_edge,
                                       double tolerance, OutputIt output) {
  if (first == last) {
    *output++ = *last;
    return;
  }

  const auto edge = to_edge(to_vertex(*first), to_vertex(*last));
  auto farthest = std::max_element(
      first, last, [&edge, &to_vertex](const auto& lhs, const auto& rhs) {
        return (edge.distance(to_vertex(lhs)) < edge.distance(to_vertex(rhs)));
      });
  if (edge.distance(to_vertex(*farthest)) > tolerance) {
    ApplyDouglasPeuckerSimplification(first, farthest, to_vertex,
                                      to_edge, tolerance, output);
    ApplyDouglasPeuckerSimplification(farthest + 1, last, to_vertex,
                                      to_edge, tolerance, output);
  } else {
    *output++ = *first;
    *output++ = *last;
  }
}


template <typename T>
using ParametrizedLine3 = Eigen::ParametrizedLine<T, 3>;


// Simplifies the @p mesh faces' contour referred by the given
// @p contour_indices by elimination of redundant vertices, i.e.
// vertices that lie within one @p tolerance distance, in meters,
// from the line that the their following and preceding vertices
// subtend.
std::vector<FaceVertexIndex> SimplifyMeshFacesContour(
    const GeoMesh& mesh,
    const std::vector<FaceVertexIndex>& contour_indices,
    double tolerance) {
  const int contour_vertex_count = contour_indices.size();
  if (contour_vertex_count <= 3) return contour_indices;
  std::vector<FaceVertexIndex> simplified_contour_indices;
  ApplyDouglasPeuckerSimplification(
      contour_indices.begin(), contour_indices.end() - 1,
      [&mesh](const FaceVertexIndex& face_vertex_index) {
        return GetMeshFaceVertexPosition(
            mesh, MeshFaceVertexAt(mesh, face_vertex_index));
      }, [](const Vector3<double>& start_vertex,
            const Vector3<double>& end_vertex) {
        return ParametrizedLine3<double>{
          start_vertex, (end_vertex - start_vertex).normalized()};
      }, tolerance, std::back_inserter(simplified_contour_indices));
  return simplified_contour_indices;
}


// Merges all the faces in the given @p mesh referred by
// @p mergeable_faces_indices into a single GeoFace.
// @param mesh Mesh the faces to be merged belong to.
// @param mergeable_faces_indices Indices of the faces in
//                                @p mesh to be merged.
// @param adjacent_faces_map Mapping of adjacent faces for the faces
//                           referred in @p mergeable_faces_indices.
// @param tolerance For contour simplification, in meters. See
//                  Simplify<eshFacesContour() function for further
//                  details.
// @returns Merged mesh faces as a single GeoFace.
// @pre The union of the all the faces referred by the given
//      @p mergeable_faces_indices expands a simply connected
//      region (i.e. with no holes).
GeoFace MergeMeshFaces(const GeoMesh& mesh,
                       const std::set<int>& mergeable_faces_indices,
                       const FaceAdjacencyMap& adjacent_faces_map,
                       double tolerance) {
  // Computes mergeable faces contour.
  const std::vector<FaceVertexIndex> contour_indices =
      SimplifyMeshFacesContour(mesh, ComputeMeshFacesContour(
          mergeable_faces_indices, adjacent_faces_map), tolerance);
  // Builds merged face by collecting all vertices in computed contour.
  GeoFace merged_face;
  const std::vector<const GeoVertex*>& vertices = mesh.vertices();
  const std::vector<const GeoNormal*>& normals = mesh.normals();
  for (const FaceVertexIndex& face_vertex_index : contour_indices) {
    const IndexFace::Vertex& face_vertex =
        MeshFaceVertexAt(mesh, face_vertex_index);
    merged_face.push_vn(*vertices[face_vertex.vertex_index],
                        *normals[face_vertex.normal_index]);
  }
  return merged_face;
}


// Simplifies a mesh by merging adjacent coplanar faces.
// @param input_mesh Mesh to be simplified.
// @param tolerance Tolerance for simplification, in meters. See
//                  MergeMeshFaces() and AggregateAdjacentCoplanarMeshFaces()
//                  functions for further details.
// @returns Output, simplified mesh.
GeoMesh SimplifyMeshFaces(const GeoMesh& input_mesh, double tolerance) {
  GeoMesh output_mesh;
  std::set<int> visited_faces_indices;
  const FaceAdjacencyMap adjacent_faces_map =
      ComputeFaceAdjacencyMap(input_mesh.faces());
  const int faces_count = static_cast<int>(input_mesh.faces().size());
  for (int face_index = 0; face_index < faces_count; ++face_index) {
    if (visited_faces_indices.count(face_index) > 0) {
      continue;
    }

    const std::set<int> mergeable_faces_indices =
        AggregateAdjacentCoplanarMeshFaces(
            input_mesh, face_index, adjacent_faces_map,
            tolerance, &visited_faces_indices);

    output_mesh.PushFace(MergeMeshFaces(
        input_mesh, mergeable_faces_indices,
        adjacent_faces_map, tolerance));
  }
  return output_mesh;
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
    if (features.simplify_mesh_faces) {
      asphalt_mesh->AddFacesFrom(SimplifyMeshFaces(
          driveable_mesh, base_grid_unit / 50.));
    } else {
      asphalt_mesh->AddFacesFrom(driveable_mesh);
    }
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
    if (features.simplify_mesh_faces) {
      h_bounds_mesh->AddFacesFrom(SimplifyMeshFaces(
          upper_h_bounds_mesh, base_grid_unit / 50.));
      h_bounds_mesh->AddFacesFrom(SimplifyMeshFaces(
          lower_h_bounds_mesh, base_grid_unit / 50.));
    } else {
      h_bounds_mesh->AddFacesFrom(upper_h_bounds_mesh);
      h_bounds_mesh->AddFacesFrom(lower_h_bounds_mesh);
    }
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
      if (features.simplify_mesh_faces) {
        lane_mesh->AddFacesFrom(SimplifyMeshFaces(
            haze_mesh, grid_unit / 50.));
      } else {
        lane_mesh->AddFacesFrom(haze_mesh);
      }
    }
    if (features.draw_stripes) {
      GeoMesh stripes_mesh;
      StripeLaneBounds(&stripes_mesh, lane, grid_unit,
                       features.stripe_elevation,
                       features.stripe_width);
      if (features.simplify_mesh_faces) {
        marker_mesh->AddFacesFrom(SimplifyMeshFaces(
            stripes_mesh, grid_unit / 50.));
      } else {
        marker_mesh->AddFacesFrom(stripes_mesh);
      }
    }
    if (features.draw_arrows) {
      GeoMesh arrows_mesh;
      MarkLaneEnds(&arrows_mesh, lane, grid_unit,
                   features.arrow_elevation);
      if (features.simplify_mesh_faces) {
        marker_mesh->AddFacesFrom(SimplifyMeshFaces(
            arrows_mesh, grid_unit / 50.));
      } else {
        marker_mesh->AddFacesFrom(arrows_mesh);
      }
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
