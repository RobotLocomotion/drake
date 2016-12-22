#include "drake/automotive/maliput/utility/generate_obj.h"

#include <cstddef>
#include <fstream>
#include <functional>
#include <initializer_list>
#include <iostream>
#include <unordered_map>
#include <vector>

#include "drake/automotive/maliput/api/junction.h"
#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/api/segment.h"

#include "drake/common/drake_assert.h"

namespace drake {
namespace maliput {
namespace utility {

namespace {

template <class T, class Hash, class KeyEqual>
class UniqueIndexer {
 public:
  UniqueIndexer() {}

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

  const std::vector<const T*>& vector() const { return vector_; }

 private:
  std::unordered_map<T, int, Hash, KeyEqual> map_;
  std::vector<const T*> vector_;
};


// A GEO-space (world-frame) vertex.
class GeoVertex {
 public:
  struct Hash {
    size_t operator()(const GeoVertex& gv) const {
      const size_t hx(std::hash<double>()(gv.v().x));
      const size_t hy(std::hash<double>()(gv.v().y));
      const size_t hz(std::hash<double>()(gv.v().z));
      return hx ^ (hy << 1) ^ (hz << 2);
    }
  };

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
  struct Hash {
    size_t operator()(const GeoNormal& gn) const {
      const size_t hx(std::hash<double>()(gn.n().x));
      const size_t hy(std::hash<double>()(gn.n().y));
      const size_t hz(std::hash<double>()(gn.n().z));
      return hx ^ (hy << 1) ^ (hz << 2);
    }
  };

  struct Equiv {
    bool operator()(const GeoNormal& lhs, const GeoNormal& rhs) const {
      return ((lhs.n().x == rhs.n().x) &&
              (lhs.n().y == rhs.n().y) &&
              (lhs.n().z == rhs.n().z));
    }
  };

  GeoNormal() {}

  GeoNormal(const api::GeoPosition& v0, const api::GeoPosition& v1)
      : n_({v1.x - v0.x, v1.y - v0.y, v1.z - v0.z}) {}

  const api::GeoPosition& n() const { return n_; }

 private:
  api::GeoPosition n_;
};


// A GEO-space (world-frame) face.
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


class IndexFace {
 public:
  struct Vertex {
    int vertex_index{};
    int normal_index{};
  };

  void push_vertex(Vertex vertex) { vertices_.push_back(vertex); }

  const std::vector<Vertex>& vertices() const { return vertices_; }

 private:
  std::vector<Vertex> vertices_;
};


class GeoMesh {
 public:
  GeoMesh() {}

  void PushFace(const GeoFace& geo_face) {
    IndexFace face;
    for (size_t gi = 0; gi < geo_face.vertices().size(); ++gi) {
      int vi = vertices_.push_back(geo_face.vertices()[gi]);
      int ni = normals_.push_back(geo_face.normals()[gi]);
      face.push_vertex({vi, ni});
    }
    faces_.push_back(face);
  }


  std::tuple<int, int>
  DumpObj(std::ostream& os, const std::string& material,
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


class SrhFace {
 public:
  SrhFace(const std::initializer_list<api::LanePosition> srh) : v_(srh) {}

  const std::vector<api::LanePosition>& v() const { return v_; }

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


void CoverLaneWithQuads(GeoMesh* mesh, const api::Lane* lane,
                        const double grid_unit) {
  const double s_max = lane->length();
  for (double s0 = 0; s0 < s_max; s0 += grid_unit) {
    double s1 = s0 + grid_unit;
    if (s1 > s_max) { s1 = s_max; }

    api::RBounds rb0 = lane->lane_bounds(s0);
    api::RBounds rb1 = lane->lane_bounds(s1);

    // TODO(maddog)  Go back to api::RoadGeometry and assert that lane-bounds
    //               always straddle r=0.  E.g., it should be nonsense if
    //               the r=0 centerline is not within the bounds of the lane.

    // Left side of lane.
    {
      double r00 = 0.;
      double r10 = 0.;
      DRAKE_DEMAND(rb0.r_min <= r00);
      DRAKE_DEMAND(rb1.r_min <= r10);
      while ((r00 < rb0.r_max) && (r10 < rb1.r_max)) {
        double r01 = r00 + grid_unit;
        double r11 = r10 + grid_unit;

        if (r01 > rb0.r_max) { r01 = rb0.r_max; }
        if (r11 > rb1.r_max) { r11 = rb0.r_max; }

        SrhFace srh_face(
            {{s0, r00, 0.}, {s1, r10, 0.}, {s1, r11, 0.}, {s0, r01, 0.}});
        mesh->PushFace(srh_face.ToGeoFace(lane));

        r00 += grid_unit;
        r10 += grid_unit;
      }
    }
    // Right side of lane.
    {
      double r00 = 0.;
      double r10 = 0.;
      DRAKE_DEMAND(rb0.r_max >= r00);
      DRAKE_DEMAND(rb1.r_max >= r10);
      while ((r00 > rb0.r_min) && (r10 > rb1.r_min)) {
        double r01 = r00 - grid_unit;
        double r11 = r10 - grid_unit;

        if (r01 < rb0.r_min) { r01 = rb0.r_min; }
        if (r11 < rb1.r_min) { r11 = rb0.r_min; }

        SrhFace srh_face(
                 {{s0, r00, 0.}, {s0, r01, 0.}, {s1, r11, 0.}, {s1, r10, 0.}});
        mesh->PushFace(srh_face.ToGeoFace(lane));

        r00 -= grid_unit;
        r10 -= grid_unit;
      }
    }
  }
}


void StripeLaneBounds(GeoMesh* mesh, const api::Lane* lane,
                      const double grid_unit) {
  const double kStripeWidth = 0.25;
  const double kStripeElevation = 0.02;

  const double half_stripe = 0.5 * kStripeWidth;

  const double s_max = lane->length();
  for (double s0 = 0; s0 < s_max; s0 += grid_unit) {
    double s1 = s0 + grid_unit;
    if (s1 > s_max) { s1 = s_max; }

    api::RBounds rb0 = lane->lane_bounds(s0);
    api::RBounds rb1 = lane->lane_bounds(s1);

    // Left side of lane.
    {
      SrhFace srh_face({
          {s0, rb0.r_max - half_stripe, kStripeElevation},
          {s1, rb1.r_max - half_stripe, kStripeElevation},
          {s1, rb1.r_max + half_stripe, kStripeElevation},
          {s0, rb0.r_max + half_stripe, kStripeElevation}});
      mesh->PushFace(srh_face.ToGeoFace(lane));
    }
    // Right side of lane.
    {
      SrhFace srh_face({
          {s0, rb0.r_min - half_stripe, kStripeElevation},
          {s1, rb1.r_min - half_stripe, kStripeElevation},
          {s1, rb1.r_min + half_stripe, kStripeElevation},
          {s0, rb0.r_min + half_stripe, kStripeElevation}});
      mesh->PushFace(srh_face.ToGeoFace(lane));
    }
  }
}

}  // namespace


void GenerateObjFile(const api::RoadGeometry* rg,
                     const std::string& dirpath,
                     const std::string& fileroot,
                     const double grid_unit) {
  GeoMesh asphalt_mesh;
  GeoMesh stripe_mesh;

  // Walk the network.
  for (int ji = 0; ji < rg->num_junctions(); ++ji) {
    const api::Junction* junction = rg->junction(ji);
    for (int si = 0; si < junction->num_segments(); ++si) {
      const api::Segment* segment = junction->segment(si);
      // TODO(maddog) We should be doing "cover segment with quads" instead,
      //              using the driveable-bounds from any lane, and then
      //              going back and using lane-bounds to paint stripes.
      for (int li = 0; li < segment->num_lanes(); ++li) {
        const api::Lane* lane = segment->lane(li);
        CoverLaneWithQuads(&asphalt_mesh, lane, grid_unit);
        StripeLaneBounds(&stripe_mesh, lane, grid_unit);
      }
    }
  }

  const std::string kYellowPaint("yellow_paint");
  const std::string kBlandAsphalt("bland_asphalt");

  const std::string obj_filename = fileroot + ".obj";
  const std::string mtl_filename = fileroot + ".mtl";

  // Create the requested OBJ file.
  {
    std::ofstream os(dirpath + "/" + obj_filename, std::ios::binary);
    os << "# GENERATED BY maliput::utility::GenerateObjFile()\n"
       << "#\n"
       << "# DON'T BE A HERO.  Do not edit by hand.\n"
       << "\n"
       << "mtllib " << mtl_filename << "\n"
       << "\n";
    int vertex_index_offset = 0;
    int normal_index_offset = 0;
    std::tie(vertex_index_offset, normal_index_offset) =
        asphalt_mesh.DumpObj(os, kBlandAsphalt,
                             vertex_index_offset, normal_index_offset);
    std::tie(vertex_index_offset, normal_index_offset) =
        stripe_mesh.DumpObj(os, kYellowPaint,
                            vertex_index_offset, normal_index_offset);
  }

  // Create the MTL file referenced by the OBJ file.
  {
    std::ofstream os(dirpath + "/" + mtl_filename, std::ios::binary);
    os << "# GENERATED BY maliput::utility::GenerateObjFile()\n"
       << "#\n"
       << "# DON'T BE A HERO.  Do not edit by hand.\n"
       << "\n"
       << "newmtl " << kYellowPaint << "\n"
       << "Ka 0.8 0.8 0.0\n"
       << "Kd 1.0 1.0 0.0\n"
       << "Ks 1.0 1.0 0.5\n"
       << "Ns 10.0\n"
       << "illum 2\n"
       << "\n"
       << "\n"
       << "newmtl " << kBlandAsphalt << "\n"
       << "Ka 0.1 0.1 0.1\n"
       << "Kd 0.1 0.1 0.1\n"
       << "Ks 0.5 0.5 0.5\n"
       << "Ns 10.0\n"
       << "illum 2\n";
  }
}


}  // namespace utility
}  // namespace maliput
}  // namespace drake
