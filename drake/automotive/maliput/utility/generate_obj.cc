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

template <class T> struct LocalHash;

template <class T>
class IndexMap {
 public:
  IndexMap() {}

  int push_back(const T& thing) {
    auto mi = map_.find(thing);
    if (mi != map_.end()) {
      // std::cerr << "found at " << mi->second << "  " << std::endl;
      // thing.printt(std::cerr);
      return mi->second;
    }
    int index = vec_.size();
    // std::cerr << "not found, now at " << index << "  " << std::endl;
    // thing.printt(std::cerr);
    map_[thing] = index;
    vec_.push_back(thing);
    return index;
  }

  const std::vector<T>& vector() const { return vec_; }

 private:
  std::unordered_map<T, int, LocalHash<T>> map_;
  std::vector<T> vec_;
};

struct GeoVertex {
  GeoVertex() {}

  explicit GeoVertex(const api::GeoPosition& _v) : v(_v) {}

  void printt(std::ostream& os) const {
    os << "v:" << v.x << " " << v.y << " " << v.z << std::endl;
  }

  api::GeoPosition v;
};

bool operator==(const GeoVertex& lhs, const GeoVertex& rhs) {
  return ((lhs.v.x == rhs.v.x) &&
          (lhs.v.y == rhs.v.y) &&
          (lhs.v.z == rhs.v.z));
}



struct GeoNormal {
  GeoNormal() {}

  GeoNormal(const api::GeoPosition& v0, const api::GeoPosition& v1)
      : n({v1.x - v0.x, v1.y - v0.y, v1.z - v0.z}) {}

  void printt(std::ostream& os) const {
    os << "vn: " << n.x << " " << n.y << " " << n.z << std::endl;
  }

  api::GeoPosition n;
};

bool operator==(const GeoNormal& lhs, const GeoNormal& rhs) {
  return ((lhs.n.x == rhs.n.x) &&
          (lhs.n.y == rhs.n.y) &&
          (lhs.n.z == rhs.n.z));
}




template<> struct LocalHash<GeoVertex> {
  typedef GeoVertex argument_type;
  typedef size_t result_type;
  result_type operator()(const argument_type& gv) const {
    const result_type hx(std::hash<double>()(gv.v.x));
    const result_type hy(std::hash<double>()(gv.v.y));
    const result_type hz(std::hash<double>()(gv.v.z));
    return hx ^ (hy << 1) ^ (hz << 2);
  }
};
template<> struct LocalHash<GeoNormal> {
  typedef GeoNormal argument_type;
  typedef size_t result_type;
  result_type operator()(const argument_type& gn) const {
    const result_type hx(std::hash<double>()(gn.n.x));
    const result_type hy(std::hash<double>()(gn.n.y));
    const result_type hz(std::hash<double>()(gn.n.z));
    return hx ^ (hy << 1) ^ (hz << 2);
  }
};


struct GeoFace {
  GeoFace() {}

  GeoFace(const std::vector<GeoVertex>& avs,
          const std::vector<GeoNormal>& ans)
      : vs(avs), ns(ans) {
    DRAKE_DEMAND(vs.size() == ns.size());
  }

  std::vector<GeoVertex> vs;
  std::vector<GeoNormal> ns;
};




struct IndexVertexWithNormal {
  int vertex_index{};
  int normal_index{};
};

struct IndexFace {
  std::vector<IndexVertexWithNormal> vns;
};


class ObjData {
 public:
  ObjData() {}

  void PushFace(const GeoFace& geo_face) {
    IndexFace face;
    for (size_t gi = 0; gi < geo_face.vs.size(); ++gi) {
      int vi = vertices_.push_back(geo_face.vs[gi]);
      int ni = normals_.push_back(geo_face.ns[gi]);
      face.vns.push_back({vi, ni});
    }
    faces_.push_back(face);
  }


  void Dump(std::ostream& os, const std::string& material) {
    os << "# Vertices" << std::endl;
    for (const GeoVertex& gv : vertices_.vector()) {
      os << "v " << gv.v.x << " " << gv.v.y << " " << gv.v.z << std::endl;
    }
    os << "# Normals" << std::endl;
    for (const GeoNormal& gn : normals_.vector()) {
      os << "vn " << gn.n.x << " " << gn.n.y << " " << gn.n.z << std::endl;
    }
    os << std::endl;
    os << "# Faces" << std::endl;
    if (!material.empty()) {
      os << "usemtl " << material << std::endl;
    }
    for (const IndexFace& f : faces_) {
      os << "f";
      for (const IndexVertexWithNormal& ivwn : f.vns) {
        os << " " << (ivwn.vertex_index + 1) << "//" << (ivwn.normal_index + 1);
      }
      os << std::endl;
    }
  }

 private:
  IndexMap<GeoVertex> vertices_;
  IndexMap<GeoNormal> normals_;
  std::vector<IndexFace> faces_;
};


struct SRPos {
  SRPos(const double as, const double ar) : s(as), r(ar) {}

  double s{};
  double r{};
};


struct SRFace {
  SRFace(const std::initializer_list<SRPos> asr) : sr(asr) {}

  std::vector<SRPos> sr;
};


void PushFace(ObjData* obj, const api::Lane* lane, const SRFace srface) {
  GeoFace geoface;
  for (const SRPos& sr : srface.sr) {
    api::GeoPosition v0(lane->ToGeoPosition({sr.s, sr.r, 0.}));
    api::GeoPosition v1(lane->ToGeoPosition({sr.s, sr.r, 1.}));
    geoface.vs.push_back(GeoVertex(v0));
    geoface.ns.push_back(GeoNormal(v0, v1));
  }
  obj->PushFace(geoface);
}


void CoverLaneWithQuads(ObjData* obj, const api::Lane* lane,
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

        PushFace(obj, lane, {{s0, r00}, {s1, r10}, {s1, r11}, {s0, r01}});

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

        PushFace(obj, lane, {{s0, r00}, {s0, r01}, {s1, r11}, {s1, r10}});

        r00 -= grid_unit;
        r10 -= grid_unit;
      }
    }
  }
}

}  // namespace


void GenerateObjFile(const api::RoadGeometry* rg,
                     const std::string& dirpath,
                     const std::string& fileroot,
                     const double grid_unit) {
  ObjData obj;

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
        CoverLaneWithQuads(&obj, lane, grid_unit);
      }
    }
  }

  const std::string kYellowPaint("yellow_paint");
  const std::string kBlandAsphalt("bland_asphalt");

  const std::string obj_filename = fileroot + ".obj";
  const std::string mtl_filename = fileroot + ".mtl";

  // Create the requested OBJ file.
  {
    std::ofstream os(dirpath + "/" + obj_filename);
    os << "# GENERATED BY maliput::utility::GenerateObjFile()" << std::endl;
    os << "#" << std::endl;
    os << "# DON'T BE A HERO, do not edit by hand." << std::endl;
    os << std::endl;
    os << "mtllib " << mtl_filename << std::endl;
    os << std::endl;
    obj.Dump(os, kBlandAsphalt);
  }

  // Create the MTL file referenced by the OBJ file.
  {
    std::ofstream os(dirpath + "/" + mtl_filename);
    os << "# GENERATED BY maliput::utility::GenerateObjFile()" << std::endl;
    os << "#" << std::endl;
    os << "# DON'T BE A HERO, do not edit by hand." << std::endl;
    os << std::endl;
    os << "newmtl " << kYellowPaint << std::endl;
    os << "Ka 0.8 0.8 0.0" << std::endl;
    os << "Kd 1.0 1.0 0.0" << std::endl;
    os << "Ks 1.0 1.0 0.5" << std::endl;
    os << "Ns 10.0" << std::endl;
    os << "illum 2" << std::endl;
    os << std::endl;
    os << std::endl;
    os << "newmtl " << kBlandAsphalt << std::endl;
    os << "Ka 0.1 0.1 0.1" << std::endl;
    os << "Kd 0.1 0.1 0.1" << std::endl;
    os << "Ks 0.5 0.5 0.5" << std::endl;
    os << "Ns 10.0" << std::endl;
    os << "illum 2" << std::endl;
    os << std::endl;
    os << std::endl;
  }
}


}  // namespace utility
}  // namespace maliput
}  // namespace drake
