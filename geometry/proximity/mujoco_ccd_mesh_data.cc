#include "drake/geometry/proximity/mujoco_ccd_mesh_data.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <utility>

#include "drake/common/drake_throw.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

using Eigen::Vector3d;

/* MuJoCo's coplanarity tolerance: faces whose normals fall into the same
 0.01-radian spherical-angle bin are merged into one polygon (see kAngleTol in
 mjCMesh::MakePolygons()). */
constexpr double kAngleTol = 0.01;

/* One bucket of (near-)coplanar faces, replicating MuJoCo's MeshPolygon
 class. Merged faces do not necessarily share edges when inserted, so edges
 are grouped into "islands" until later insertions connect them; each island
 that survives becomes one output polygon. */
struct NormalBucket {
  /* The directed boundary edges of the merged region so far. An inserted
   face cancels edges that its own (reversed) edges match. */
  std::vector<std::pair<int, int>> edges;
  /* islands[i] is the island id of edges[i]. */
  std::vector<int> islands;
  int num_islands{0};
};

/* Replicates MuJoCo's mjuu_makenormal(): the unit normal of the triangle
 (a, b, c), falling back to (1, 0, 0) when the triangle is degenerate. MuJoCo
 assigns each merged polygon the normal its first three vertices produce (see
 mjCMesh::MakePolygonNormals(), which overwrites the provisional bin-center
 normals that MakePolygons() records). */
Vector3d MakeNormal(const Vector3d& a, const Vector3d& b, const Vector3d& c) {
  const Vector3d normal = (b - a).cross(c - a);
  const double norm = normal.norm();
  constexpr double kMjEps = 1e-14;  // MuJoCo's mjEPS.
  if (norm < kMjEps) {
    return Vector3d::UnitX();
  }
  return normal / norm;
}

/* Replicates MuJoCo's MeshPolygonKey(): quantizes the direction of a unit
 normal to 0.01-radian bins of its spherical angles. Returns the bin as
 integral-valued doubles (theta_bin, phi_bin). */
std::pair<double, double> QuantizedNormalKey(const Vector3d& n_in) {
  // Adding 0.0 turns -0.0 into +0.0 so that atan2 is insensitive to the sign
  // of zero, exactly as MuJoCo does.
  const double nx = n_in.x() + 0.0;
  const double ny = n_in.y() + 0.0;
  const double nz = n_in.z() + 0.0;
  if (std::abs(nz) > 1.0 - 1e-7) {
    const double rphi = nz < 0 ? std::round(M_PI / kAngleTol) : 0.0;
    return {0.0, rphi};
  }
  const double rtheta = std::round(std::atan2(ny, nx) / kAngleTol);
  const double rphi = std::round(std::acos(nz) / kAngleTol);
  return {rtheta, rphi};
}

/* Replicates MuJoCo's MeshPolygon::CombineIslands(): renumbers the islands
 when a newly inserted face connects `island1` and `island2`, keeping the
 smaller index (returned via island1) and compacting indices above the
 removed one. */
void CombineIslands(NormalBucket* bucket, int* island1, int island2) {
  if (island2 < *island1) {
    std::swap(*island1, island2);
  }
  for (int& island : bucket->islands) {
    if (island == island2) {
      island = *island1;
    } else if (island > island2) {
      --island;
    }
  }
}

/* Inserts one face (given as its CCW vertex loop) into the bucket. This
 generalizes MuJoCo's MeshPolygon::InsertFace() from triangles to n-gons:
 each directed edge of the face either cancels the reversed edge already on
 some island's boundary (joining that island) or is added to the boundary. */
void InsertFace(NormalBucket* bucket, const std::vector<int>& loop) {
  const int n = std::ssize(loop);
  int island = -1;
  std::vector<std::pair<int, int>> new_edges;
  for (int i = 0; i < n; ++i) {
    const int a = loop[i];
    const int b = loop[(i + 1) % n];
    bool cancelled = false;
    for (int j = 0; j < std::ssize(bucket->edges); ++j) {
      if (bucket->edges[j].first == b && bucket->edges[j].second == a) {
        const int other = bucket->islands[j];
        bucket->edges.erase(bucket->edges.begin() + j);
        bucket->islands.erase(bucket->islands.begin() + j);
        if (island == -1) {
          island = other;
        } else if (other != island) {
          --bucket->num_islands;
          CombineIslands(bucket, &island, other);
        }
        cancelled = true;
        break;
      }
    }
    if (!cancelled) new_edges.push_back({a, b});
  }
  if (island == -1) island = bucket->num_islands++;
  for (const auto& edge : new_edges) {
    bucket->edges.push_back(edge);
    bucket->islands.push_back(island);
  }
}

/* Replicates MuJoCo's MeshPolygon::Paths(): traces each island's directed
 boundary edges into a closed vertex loop. On a closed convex hull each
 island's edges form exactly one cycle, so simple successor-following
 recovers it.

 Known deliberate divergence: when a multi-face bucket boils down to exactly
 three surviving edges that happen to sit in the edge list out of cyclic
 order, MuJoCo's Paths() shortcut emits them in storage order (which can
 yield a reversed-winding triangle); this implementation always follows
 successors and emits the correctly wound loop. */
std::vector<std::vector<int>> TracePaths(const NormalBucket& bucket) {
  std::vector<std::vector<int>> paths;
  const int num_edges = std::ssize(bucket.edges);
  for (int i = 0; i < bucket.num_islands; ++i) {
    std::vector<int> path;
    for (int j = 0; j < num_edges; ++j) {
      if (bucket.islands[j] == i) {
        path.push_back(bucket.edges[j].first);
        path.push_back(bucket.edges[j].second);
        break;
      }
    }
    if (path.empty()) continue;
    // Follow directed edges until the cycle closes (bounded by num_edges
    // steps to be robust to malformed input).
    for (int steps = 0; steps < num_edges; ++steps) {
      const int next = path.back();
      bool advanced = false;
      for (int j = 0; j < num_edges; ++j) {
        if (bucket.islands[j] == i && bucket.edges[j].first == next) {
          if (bucket.edges[j].second != path.front()) {
            path.push_back(bucket.edges[j].second);
            advanced = true;
          }
          break;
        }
      }
      if (!advanced) break;
    }
    paths.push_back(std::move(path));
  }
  return paths;
}

}  // namespace

MujocoCcdMeshData MakeMujocoCcdMeshData(
    const PolygonSurfaceMesh<double>& hull) {
  DRAKE_THROW_UNLESS(hull.num_vertices() >= 4);
  MujocoCcdMeshData data;

  // Vertices (single precision, as MuJoCo stores them) and centroid.
  data.nvert = hull.num_vertices();
  data.vert.reserve(3 * data.nvert);
  Vector3d centroid = Vector3d::Zero();
  for (int v = 0; v < data.nvert; ++v) {
    const Vector3d& p = hull.vertex(v);
    data.vert.push_back(static_cast<float>(p.x()));
    data.vert.push_back(static_cast<float>(p.y()));
    data.vert.push_back(static_cast<float>(p.z()));
    centroid += p;
  }
  data.centroid = centroid / data.nvert;

  // Detect an inward-wound input (e.g. a hull whose face topology was cached
  // at one scale and re-instantiated at a mirroring scale, which flips the
  // loops' orientation): the signed volume enclosed by an outward-wound
  // closed surface is positive. When inverted, every face loop is reversed on
  // insertion so the tables are always outward-wound, as MuJoCo requires.
  double signed_volume = 0.0;
  for (int f = 0; f < hull.num_faces(); ++f) {
    const SurfacePolygon face = hull.element(f);
    const Vector3d& v0 = hull.vertex(face.vertex(0));
    for (int i = 1; i + 1 < face.num_vertices(); ++i) {
      signed_volume += v0.dot(hull.vertex(face.vertex(i))
                                  .cross(hull.vertex(face.vertex(i + 1)))) /
                       6.0;
    }
  }
  const bool reverse_winding = signed_volume < 0.0;

  // Bucket the faces by quantized normal direction. A std::map keyed on the
  // integral bin values gives deterministic output ordering (MuJoCo uses an
  // unordered_map here; bucket *content* is identical, only the order of the
  // output polygons may differ, which has no semantic effect).
  std::map<std::pair<double, double>, NormalBucket> buckets;
  for (int f = 0; f < hull.num_faces(); ++f) {
    const Vector3d normal =
        reverse_winding ? (-hull.face_normal(f)).eval() : hull.face_normal(f);
    if (!normal.allFinite() || normal.norm() < 0.5) continue;  // Degenerate.
    const std::pair<double, double> key = QuantizedNormalKey(normal);
    NormalBucket& bucket = buckets[key];
    const SurfacePolygon face = hull.element(f);
    const int n = face.num_vertices();
    std::vector<int> loop(n);
    for (int i = 0; i < n; ++i) {
      loop[i] = face.vertex(reverse_winding ? n - 1 - i : i);
    }
    InsertFace(&bucket, loop);
  }

  // Trace each bucket's islands into polygons and flatten the tables.
  std::vector<std::vector<int>> vertex_polygons(data.nvert);
  for (const auto& key_bucket : buckets) {
    const NormalBucket& bucket = key_bucket.second;
    for (std::vector<int>& path : TracePaths(bucket)) {
      const int path_size = static_cast<int>(std::ssize(path));
      if (path_size < 3) continue;
      const int poly_index = data.polynum++;
      // The polygon's normal comes from its first three vertices, exactly as
      // MuJoCo's mjCMesh::MakePolygonNormals() computes it.
      const Vector3d normal = MakeNormal(
          hull.vertex(path[0]), hull.vertex(path[1]), hull.vertex(path[2]));
      data.polynormal.push_back(normal.x());
      data.polynormal.push_back(normal.y());
      data.polynormal.push_back(normal.z());
      data.polyvertadr.push_back(static_cast<int>(std::ssize(data.polyvert)));
      data.polyvertnum.push_back(path_size);
      data.npolygonmax = std::max(data.npolygonmax, path_size);
      for (int v : path) {
        DRAKE_THROW_UNLESS(0 <= v && v < data.nvert);
        data.polyvert.push_back(v);
        vertex_polygons[v].push_back(poly_index);
      }
    }
  }

  // Flatten the vertex-to-polygon adjacency.
  data.polymapadr.reserve(data.nvert);
  data.polymapnum.reserve(data.nvert);
  for (int v = 0; v < data.nvert; ++v) {
    const int degree = static_cast<int>(std::ssize(vertex_polygons[v]));
    data.polymapadr.push_back(static_cast<int>(std::ssize(data.polymap)));
    data.polymapnum.push_back(degree);
    data.nmeshdegmax = std::max(data.nmeshdegmax, degree);
    data.polymap.insert(data.polymap.end(), vertex_polygons[v].begin(),
                        vertex_polygons[v].end());
  }

  return data;
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
